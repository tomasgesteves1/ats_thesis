#include "uav_mpc/uav_mpc_node.hpp"
#include "uav_mpc/uav_mpc_kinematics.hpp"
#include "uav_mpc/uav_mpc_telemetry.hpp"
#include <limits>
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace uav_mpc {

UavMpcNode::UavMpcNode() 
    : Node("uav_mpc_node"), 
      target_initialized_(false), 
      odom_received_(false),
      vehicle_status_received_(false),
      offboard_setpoint_counter_(0),
      px4_hover_thrust_(0.618),
      current_system_id_(2),
      last_boat_horizon_time_(0, 0, RCL_ROS_TIME),
      active_tether_max_length_(-1.0),
      open_loop_test_(false),
      open_loop_active_(false),
      open_loop_step_(0) {
      
    pipeline_ = std::make_unique<UavMpcPipeline>();

    // Declare dynamic parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<bool>("open_loop_test", false);
    this->declare_parameter<bool>("use_tether", true);
    this->declare_parameter<bool>("tether_model", true);  // tether force in the prediction model
    // Align the yaw with the heading of the reference path (orientation of its first pose) instead of looking at
    // the tether attachment point (used for landing, with the boat heading as reference). Initial value; it is
    // switched at run time through the yaw_from_reference topic.
    yaw_from_reference_ = this->declare_parameter<bool>("yaw_from_reference", false);
    this->declare_parameter<double>("tether_rate_limit", 1.5);
    this->declare_parameter<double>("winch_speed_limit", 2.0);
    rcl_interfaces::msg::ParameterDescriptor tether_desc;
    tether_desc.dynamic_typing = true;
    this->declare_parameter("tether_max_length", rclcpp::ParameterValue(15.0), tether_desc);
    this->declare_parameter<std::string>("trajectory_type", "hold");
    this->declare_parameter<double>("mass", 2.06);
    this->declare_parameter<double>("gravity", 9.81);
    this->declare_parameter<double>("z_min", 2.0);
    // Lower bound on the height while the low-altitude mode is requested on the low_altitude topic (takeoff from
    // and landing on the pad, which are below z_min)
    this->declare_parameter<double>("z_min_low_altitude", 0.0);
    // Observer of the external force (offset-free tracking); its vertical component is frozen while the winch pulls
    // the UAV (topic tether_pull: the pull of takeoff and landing is opposed, not compensated; the horizontal
    // components keep rejecting wind and model errors) and the estimate is reset while disarmed
    this->declare_parameter<bool>("disturbance_observer", true);
    this->declare_parameter<double>("disturbance_gain", 0.03);
    this->declare_parameter<double>("disturbance_max_force", 8.0);
    this->declare_parameter<double>("v_xy_max", 10.0);
    this->declare_parameter<double>("v_z_max", 10.0);
    this->declare_parameter<double>("tilt_max", 0.5);
    this->declare_parameter<double>("thrust_min", 2.02);
    this->declare_parameter<double>("thrust_max", 39.96);
    this->declare_parameter<double>("hover_throttle", 0.618);
    this->declare_parameter<double>("max_pose_age", 0.05);   // s, cap of the pose age compensation
    this->declare_parameter<double>("rotor_speed_min", 150.0);
    this->declare_parameter<double>("rotor_speed_max", 1000.0);
    // The hover thrust estimated by PX4 adapts to the load carried (here including the tether pull), which the
    // NMPC model already accounts for; using it would count the tether twice, so the nominal value is used by default.
    this->declare_parameter<bool>("use_px4_hover_estimate", false);
    this->declare_parameter<double>("hold_height", 2.0);
    this->declare_parameter<double>("takeoff_height", 4.0);

    // Relative namespaces for topics (Rule 4 of CODE_STANDARDS.md)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UavMpcNode::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_position", 10, std::bind(&UavMpcNode::targetCallback, this, std::placeholders::_1));

    trajectory_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "reference_path", 10, std::bind(&UavMpcNode::trajectoryPathCallback, this, std::placeholders::_1));

    tether_length_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "tether_length", 10, std::bind(&UavMpcNode::tetherLengthCallback, this, std::placeholders::_1));

    boat_horizon_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "boat_mpc_horizon", 10, std::bind(&UavMpcNode::boatHorizonCallback, this, std::placeholders::_1));

    yaw_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "yaw_from_reference", 10, [this](std_msgs::msg::Bool::SharedPtr msg) { yaw_from_reference_ = msg->data; });

    low_altitude_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "low_altitude", 10, [this](std_msgs::msg::Bool::SharedPtr msg) { low_altitude_ = msg->data; });

    tether_pull_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "tether_pull", 10, [this](std_msgs::msg::Bool::SharedPtr msg) { tether_pull_ = msg->data; });

    // Configure QoS identical to drone_tracker for PX4 (SensorDataQoS)
    auto qos = rclcpp::SensorDataQoS();
    
    // Subscribe to estimated hover thrust from PX4
    hover_thrust_sub_ = this->create_subscription<px4_msgs::msg::HoverThrustEstimate>(
        "/px4_1/fmu/out/hover_thrust_estimate", qos, std::bind(&UavMpcNode::hoverThrustCallback, this, std::placeholders::_1));

    // Subscribe to vehicle status from PX4
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/px4_1/fmu/out/vehicle_status_v1", qos, std::bind(&UavMpcNode::vehicleStatusCallback, this, std::placeholders::_1));

    // Initialize TF buffer and listener
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", qos);
    attitude_setpoint_pub_ = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
        "/px4_1/fmu/in/vehicle_attitude_setpoint", qos);

    // Relative visualization publishers
    predicted_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "predicted_trajectory", 10);
    target_point_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "target_point", 10);
    reference_trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "reference_trajectory", 10);

    // MPC tether force publisher (relative topic)
    mpc_tether_force_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "mpc_tether_force_mag", 10);

    virtual_tether_distance_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "virtual_tether_distance", 10);

    solve_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("uav_solve_time_ms", 10);
    disturbance_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("disturbance_estimate", 10);

    virtual_tether_limit_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "virtual_tether_limit", 10);

    virtual_tether_separation_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "virtual_tether_separation_speed", 10);

    // MPC debug states and inputs publisher (relative topic)
    mpc_states_inputs_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "mpc_states_and_inputs", 10);

    // Path publishers (relative topics)
    mpc_predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "mpc_predicted_path", 10);
    mpc_reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "mpc_reference_path", 10);

    // Open-loop test publishers (relative topics)
    open_loop_predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "open_loop_predicted_path", 10);
    open_loop_actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "open_loop_actual_path", 10);

    // Timer at 50Hz (0.02s) using node's clock (sim time) to match the MPC dt
    timer_ = this->create_timer(
        std::chrono::milliseconds(20), std::bind(&UavMpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "UAV MPC Node successfully initialized.");
}

void UavMpcNode::targetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "New reference received: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    pipeline_->setReference({msg->x, msg->y, msg->z});
}

void UavMpcNode::trajectoryPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        return;
    }
    
    std::vector<TrajectoryPoint> path_points;
    const double Ts = 0.02; // Control period (s)
    size_t num_poses = msg->poses.size();
    
    path_points.resize(num_poses);
    for (size_t i = 0; i < num_poses; ++i) {
        path_points[i].px = msg->poses[i].pose.position.x;
        path_points[i].py = msg->poses[i].pose.position.y;
        path_points[i].pz = msg->poses[i].pose.position.z;
    }
    
    // Compute velocities using finite difference
    for (size_t i = 0; i < num_poses; ++i) {
        if (i < num_poses - 1) {
            path_points[i].vx = (path_points[i+1].px - path_points[i].px) / Ts;
            path_points[i].vy = (path_points[i+1].py - path_points[i].py) / Ts;
            path_points[i].vz = (path_points[i+1].pz - path_points[i].pz) / Ts;
        } else {
            if (num_poses > 1) {
                path_points[i].vx = path_points[i-1].vx;
                path_points[i].vy = path_points[i-1].vy;
                path_points[i].vz = path_points[i-1].vz;
            } else {
                path_points[i].vx = 0.0;
                path_points[i].vy = 0.0;
                path_points[i].vz = 0.0;
            }
        }
    }
    
    pipeline_->setExternalReferencePath(path_points);

    // The reference yaw travels in the orientation of the first pose (identity -> yaw 0)
    const auto& q = msg->poses.front().pose.orientation;
    yaw_ref_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void UavMpcNode::tetherLengthCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Deprecated: We now use the static/user parameter 'tether_max_length' for the MPC constraint.
    (void)msg;
}

void UavMpcNode::boatHorizonCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    last_boat_horizon_time_ = this->now();
    std::vector<std::vector<double>> boat_horizon;
    for (const auto& pose : msg->poses) {
        boat_horizon.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    latest_boat_horizon_ = boat_horizon;
    pipeline_->updateBoatHorizon(boat_horizon);
}

void UavMpcNode::hoverThrustCallback(const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
    if (msg->valid) {
        px4_hover_thrust_ = msg->hover_thrust;
    }
}

void UavMpcNode::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    latest_vehicle_status_ = *msg;
    vehicle_status_received_ = true;
    current_system_id_ = msg->system_id;
}

void UavMpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_ = *msg;
    odom_received_ = true;

    // The odometry gives the velocity of the model origin, while the controlled point (drone/base_link, used for the
    // position) sits at a fixed offset r along the body axes: its velocity is v + omega x r (body axes). Without this
    // term every attitude rotation would read as a translation of the controlled point
    if (!com_offset_known_) {
        try {
            const auto tr = tf_buffer_->lookupTransform("drone/ground_truth", "drone/base_link", tf2::TimePointZero);
            com_offset_ = {tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z};
            com_offset_known_ = true;
        } catch (const tf2::TransformException &) {
            return;
        }
    }
    const auto & w = msg->twist.twist.angular;
    const auto & r = com_offset_;
    latest_odom_.twist.twist.linear.x += w.y * r[2] - w.z * r[1];
    latest_odom_.twist.twist.linear.y += w.z * r[0] - w.x * r[2];
    latest_odom_.twist.twist.linear.z += w.x * r[1] - w.y * r[0];
}

void UavMpcNode::controlLoop() {
    open_loop_test_ = this->get_parameter("open_loop_test").as_bool();

    bool use_tether = this->get_parameter("use_tether").as_bool();
    pipeline_->setUseTether(use_tether);
    pipeline_->setTetherModel(this->get_parameter("tether_model").as_bool());
    pipeline_->setYawHeading(yaw_from_reference_, yaw_ref_);

    double target_tether_max_length = 15.0;
    auto tether_max_length_param = this->get_parameter("tether_max_length");
    if (tether_max_length_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        target_tether_max_length = static_cast<double>(tether_max_length_param.as_int());
    } else if (tether_max_length_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        target_tether_max_length = tether_max_length_param.as_double();
    }

    // Rate-limit the tether length adjustments to avoid abrupt control spikes
    if (active_tether_max_length_ < 0.0) {
        active_tether_max_length_ = target_tether_max_length;
    } else {
        double rate_limit = this->get_parameter("tether_rate_limit").as_double();
        double max_step = rate_limit * 0.02; // 50 Hz control loop (dt = 0.02s)
        double diff = target_tether_max_length - active_tether_max_length_;
        if (diff > max_step) {
            active_tether_max_length_ += max_step;
        } else if (diff < -max_step) {
            active_tether_max_length_ -= max_step;
        } else {
            active_tether_max_length_ = target_tether_max_length;
        }
    }
    pipeline_->updateTetherLength(active_tether_max_length_);

    double anchor_x = 0.0;
    double anchor_y = 0.0;
    double anchor_z = 0.0;

    if (use_tether) {
        // Timeout check for boat predicted horizon
        if (last_boat_horizon_time_.nanoseconds() > 0) {
            double elapsed_since_last_horizon = (this->now() - last_boat_horizon_time_).seconds();
            if (elapsed_since_last_horizon > 0.5) {
                // Clear the boat horizon to fall back to static anchor positioning in the OCP
                pipeline_->updateBoatHorizon(std::vector<std::vector<double>>());
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Boat predicted horizon timeout (%.2f s). Falling back to static anchor.", elapsed_since_last_horizon);
            }
        } else {
            // No message ever received, fall back to static anchor
            pipeline_->updateBoatHorizon(std::vector<std::vector<double>>());
        }

        // Look up the exact anchor position using TF (world -> boat/tether_anchor)
        try {
            auto transform = tf_buffer_->lookupTransform("world", "boat/tether_anchor", tf2::TimePointZero);
            anchor_x = transform.transform.translation.x;
            anchor_y = transform.transform.translation.y;
            anchor_z = transform.transform.translation.z;
            pipeline_->updateAnchorPosition(
                anchor_x,
                anchor_y,
                anchor_z
            );
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Could not obtain transform from world to boat/tether_anchor: %s", ex.what());
        }
    } else {
        // If disabled, keep anchor at origin
        pipeline_->updateAnchorPosition(0.0, 0.0, 0.0);
        pipeline_->updateBoatHorizon(std::vector<std::vector<double>>());
    }

    // Look up the drone COM pose (world -> drone/base_link)
    double drone_x = 0.0, drone_y = 0.0, drone_z = 0.0;
    double drone_qx = 0.0, drone_qy = 0.0, drone_qz = 0.0, drone_qw = 1.0;
    double pose_age = 0.0;  // age of the pose sample at the time of the control step (s)
    bool tf_success = false;
    
    try {
        auto transform = tf_buffer_->lookupTransform("world", "drone/base_link", tf2::TimePointZero);
        drone_x = transform.transform.translation.x;
        drone_y = transform.transform.translation.y;
        drone_z = transform.transform.translation.z;
        drone_qx = transform.transform.rotation.x;
        drone_qy = transform.transform.rotation.y;
        drone_qz = transform.transform.rotation.z;
        drone_qw = transform.transform.rotation.w;
        pose_age = (this->get_clock()->now() - rclcpp::Time(transform.header.stamp, this->get_clock()->get_clock_type())).seconds();
        tf_success = true;

        // Compute and publish virtual tether distance, limit, and separation speed for plotting/monitoring
        if (use_tether) {
            double dx = drone_x - anchor_x;
            double dy = drone_y - anchor_y;
            double dz = drone_z - anchor_z;
            double virtual_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            std_msgs::msg::Float64 dist_msg;
            dist_msg.data = virtual_dist;
            virtual_tether_distance_pub_->publish(dist_msg);

            std_msgs::msg::Float64 limit_msg;
            limit_msg.data = active_tether_max_length_;
            virtual_tether_limit_pub_->publish(limit_msg);

            if (odom_received_ && virtual_dist > 0.05) {
                double v_body[3] = {latest_odom_.twist.twist.linear.x, latest_odom_.twist.twist.linear.y, latest_odom_.twist.twist.linear.z};
                double v_world[3] = {0.0, 0.0, 0.0};
                kinematics::rotateVectorByQuaternion(drone_qx, drone_qy, drone_qz, drone_qw, v_body, v_world);

                double b_vx = 0.0, b_vy = 0.0;
                if (!latest_boat_horizon_.empty() && latest_boat_horizon_.size() >= 2) {
                    b_vx = (latest_boat_horizon_[1][0] - latest_boat_horizon_[0][0]) / 0.1;
                    b_vy = (latest_boat_horizon_[1][1] - latest_boat_horizon_[0][1]) / 0.1;
                }
                double v_sep = (dx * (v_world[0] - b_vx) + dy * (v_world[1] - b_vy) + dz * v_world[2]) / virtual_dist;
                std_msgs::msg::Float64 v_sep_msg;
                v_sep_msg.data = v_sep;
                virtual_tether_separation_speed_pub_->publish(v_sep_msg);
            }
        }
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Could not obtain transform from world to drone/base_link: %s", ex.what());
    }

    if (!tf_success || !odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Skipping control loop step: waiting for TF transform and odometry velocity.");
        return;
    }

    if (!target_initialized_) {
        double hold_height = this->get_parameter("hold_height").as_double();
        pipeline_->setReference({drone_x, drone_y, hold_height});
        target_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial target set from drone TF: X=%.2f, Y=%.2f, Z=%.2f", 
                    drone_x, drone_y, hold_height);
    }

    // Update dynamic trajectory parameters
    std::string traj_type = this->get_parameter("trajectory_type").as_string();
    if (traj_type == "external") {
        pipeline_->setTrajectoryType(TrajectoryType::EXTERNAL);
    } else {
        pipeline_->setTrajectoryType(TrajectoryType::HOLD);
    }

    // Update the run-time parameters of the controller (Table 4.12)
    UavMpcParams params;
    params.mass = this->get_parameter("mass").as_double();
    params.gravity = this->get_parameter("gravity").as_double();
    params.z_min = this->get_parameter(low_altitude_ ? "z_min_low_altitude" : "z_min").as_double();
    params.v_xy_max = this->get_parameter("v_xy_max").as_double();
    params.v_z_max = this->get_parameter("v_z_max").as_double();
    params.tilt_max = this->get_parameter("tilt_max").as_double();
    params.thrust_min = this->get_parameter("thrust_min").as_double();
    params.thrust_max = this->get_parameter("thrust_max").as_double();
    params.d_dot_max = this->get_parameter("winch_speed_limit").as_double();
    params.hover_throttle = this->get_parameter("hover_throttle").as_double();
    params.rotor_speed_min = this->get_parameter("rotor_speed_min").as_double();
    params.rotor_speed_max = this->get_parameter("rotor_speed_max").as_double();
    pipeline_->setParameters(params);
    pipeline_->configureDisturbanceObserver(this->get_parameter("disturbance_observer").as_bool(),
                                            this->get_parameter("disturbance_gain").as_double(),
                                            this->get_parameter("disturbance_max_force").as_double());
    pipeline_->setDisturbanceFrozen(tether_pull_);
    if (latest_vehicle_status_.arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
        pipeline_->resetDisturbance();
    }
    {
        const auto d = pipeline_->disturbanceEstimate();
        geometry_msgs::msg::Vector3 msg;
        msg.x = d[0];
        msg.y = d[1];
        msg.z = d[2];
        disturbance_pub_->publish(msg);
    }

    // The pose and the control loop run at the same rate without synchronisation, so the latest sample is up to one
    // period old (or repeated): the position is propagated to the current time with the measured velocity
    {
        double v_body[3] = {latest_odom_.twist.twist.linear.x, latest_odom_.twist.twist.linear.y, latest_odom_.twist.twist.linear.z};
        double v_world[3] = {0.0, 0.0, 0.0};
        kinematics::rotateVectorByQuaternion(drone_qx, drone_qy, drone_qz, drone_qw, v_body, v_world);
        const double age = std::max(0.0, std::min(pose_age, this->get_parameter("max_pose_age").as_double()));
        drone_x += v_world[0] * age;
        drone_y += v_world[1] * age;
        drone_z += v_world[2] * age;
    }

    // Inject TF pose and latest velocity into the pipeline
    std::vector<double> state = {
        drone_x,
        drone_y,
        drone_z,
        latest_odom_.twist.twist.linear.x,
        latest_odom_.twist.twist.linear.y,
        latest_odom_.twist.twist.linear.z
    };
    pipeline_->updateState(state);
    pipeline_->updateOrientation(drone_qx, drone_qy, drone_qz, drone_qw);

    double sim_time_s = this->get_clock()->now().seconds();

    // Check transition to OFFBOARD with open-loop test enabled
    if (latest_vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        if (open_loop_test_ && !open_loop_active_) {
            RCLCPP_INFO(this->get_logger(), "Transition to OFFBOARD detected with open-loop test enabled. Capturing horizon...");
            open_loop_steps_ = pipeline_->getOpenLoopHorizon(sim_time_s);
            open_loop_step_ = 0;
            open_loop_active_ = true;
            
            // Build and store the static open-loop predicted path
            auto current_time_ros = this->get_clock()->now();
            open_loop_predicted_path_.header.frame_id = "world";
            open_loop_predicted_path_.header.stamp = current_time_ros;
            open_loop_predicted_path_.poses.clear();
            if (!open_loop_steps_.empty()) {
                const auto& pred_positions = open_loop_steps_[0].predicted_positions;
                for (const auto& pos : pred_positions) {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = "world";
                    pose.header.stamp = current_time_ros;
                    pose.pose.position.x = pos[0];
                    pose.pose.position.y = pos[1];
                    pose.pose.position.z = pos[2];
                    pose.pose.orientation.w = 1.0;
                    open_loop_predicted_path_.poses.push_back(pose);
                }
            }
            
            // Clear the actual path followed
            open_loop_actual_path_.header.frame_id = "world";
            open_loop_actual_path_.header.stamp = current_time_ros;
            open_loop_actual_path_.poses.clear();
        }
    } else {
        open_loop_active_ = false;
        open_loop_step_ = 0;
        open_loop_steps_.clear();
    }

    // Execute calculations or replay open-loop commands
    UavControlOutput output;
    if (open_loop_active_ && !open_loop_steps_.empty()) {
        size_t idx = open_loop_step_;
        if (idx >= open_loop_steps_.size()) {
            idx = open_loop_steps_.size() - 1; // Hold the last control input
        }
        output = open_loop_steps_[idx];
        if (open_loop_step_ < open_loop_steps_.size()) {
            open_loop_step_++;
        }
    } else {
        const auto t_start = std::chrono::steady_clock::now();
        output = pipeline_->computeControl(sim_time_s);
        std_msgs::msg::Float64 solve_msg;
        solve_msg.data = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
        solve_time_pub_->publish(solve_msg);
    }


    // Calculate current Euler angles
    double drone_roll = 0.0, drone_pitch = 0.0, drone_yaw = 0.0;
    kinematics::quaternionToEuler(drone_qx, drone_qy, drone_qz, drone_qw, drone_roll, drone_pitch, drone_yaw);

    // Print MPC optimal commands and current attitude throttled to 1Hz
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[MPC Output] Cmd - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f° | Thrust: %.2f N",
        output.u_opt[0] * 180.0 / M_PI,
        output.u_opt[1] * 180.0 / M_PI,
        output.psi_cmd * 180.0 / M_PI,
        output.u_opt[2]);

    // Always publish OffboardControlMode to feed PX4 watchdog and allow offboard transition
    publishOffboardControlMode();

    // Always publish VehicleAttitudeSetpoint to allow offboard transition
    publishAttitudeSetpoint(output);

    // Publish RViz/Foxglove visualization markers
    publishVisualizationMarkers(output);

    // Publish estimated tether force magnitude using telemetry helper
    double force_mag = 0.0;
    if (use_tether) {
        double T0_val = telemetry::calculateWinchTension(active_tether_max_length_);
        force_mag = telemetry::estimateTetherForce(
            drone_x, drone_y, drone_z,
            anchor_x, anchor_y, anchor_z,
            T0_val
        );
    }
    std_msgs::msg::Float64 force_msg;
    force_msg.data = force_mag;
    mpc_tether_force_pub_->publish(force_msg);

    // Rotate linear velocities to world frame (ENU) for accurate velocity reference comparison
    double v_body[3] = {
        latest_odom_.twist.twist.linear.x,
        latest_odom_.twist.twist.linear.y,
        latest_odom_.twist.twist.linear.z
    };
    double v_world[3] = {0.0, 0.0, 0.0};
    kinematics::rotateVectorByQuaternion(drone_qx, drone_qy, drone_qz, drone_qw, v_body, v_world);

    // Publish MPC telemetry states and inputs
    std_msgs::msg::Float64MultiArray telemetry_msg;
    telemetry_msg.data = {
        drone_x,                           // 0: State x (m)
        drone_y,                           // 1: State y (m)
        drone_z,                           // 2: State z (m)
        latest_odom_.twist.twist.linear.x, // 3: State vx (m/s) [body]
        latest_odom_.twist.twist.linear.y, // 4: State vy (m/s) [body]
        latest_odom_.twist.twist.linear.z, // 5: State vz (m/s) [body]
        output.u_opt[0],                   // 6: Input roll cmd (rad)
        output.u_opt[1],                   // 7: Input pitch cmd (rad)
        output.u_opt[2],                   // 8: Input thrust cmd (N)
        drone_roll,                        // 9: Current roll (rad)
        drone_pitch,                       // 10: Current pitch (rad)
        drone_yaw,                         // 11: Current yaw (rad)
        output.current_reference[0],       // 12: Reference x (m)
        output.current_reference[1],       // 13: Reference y (m)
        output.current_reference[2],       // 14: Reference z (m)
        v_world[0],                        // 15: State vx world (m/s)
        v_world[1],                        // 16: State vy world (m/s)
        v_world[2],                        // 17: State vz world (m/s)
        output.current_reference_velocity[0], // 18: Reference vx world (m/s)
        output.current_reference_velocity[1], // 19: Reference vy world (m/s)
        output.current_reference_velocity[2], // 20: Reference vz world (m/s)
        output.psi_cmd,                       // 21: Input yaw cmd (rad)
        output.predicted_attitude[0],         // 22: Roll predicted 0.2 s ahead (rad)
        output.predicted_attitude[1],         // 23: Pitch predicted 0.2 s ahead (rad)
        output.predicted_attitude[2]          // 24: Yaw predicted 0.2 s ahead (rad)
    };
    
    // Add layout descriptors for readability
    std_msgs::msg::MultiArrayDimension dim;
    dim.label = "x,y,z,vx,vy,vz,roll_cmd,pitch_cmd,thrust_cmd,roll,pitch,yaw,ref_x,ref_y,ref_z,vx_world,vy_world,vz_world,ref_vx,ref_vy,ref_vz,yaw_cmd,roll_pred,pitch_pred,yaw_pred";
    dim.size = telemetry_msg.data.size();
    dim.stride = telemetry_msg.data.size();
    telemetry_msg.layout.dim.push_back(dim);
    
    mpc_states_inputs_pub_->publish(telemetry_msg);

    // Publish MPC Predicted Path (nav_msgs/msg/Path)
    auto current_time = this->get_clock()->now();
    nav_msgs::msg::Path pred_path_msg;
    pred_path_msg.header.frame_id = "world";
    pred_path_msg.header.stamp = current_time;
    
    for (const auto& pos : output.predicted_positions) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = current_time;
        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        pose.pose.position.z = pos[2];
        pose.pose.orientation.w = 1.0;
        pred_path_msg.poses.push_back(pose);
    }
    mpc_predicted_path_pub_->publish(pred_path_msg);

    // Publish MPC Reference Path (nav_msgs/msg/Path)
    nav_msgs::msg::Path ref_path_msg;
    ref_path_msg.header.frame_id = "world";
    ref_path_msg.header.stamp = current_time;
    
    for (const auto& pos : output.reference_path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = current_time;
        pose.pose.position.x = pos[0];
        pose.pose.position.y = pos[1];
        pose.pose.position.z = pos[2];
        pose.pose.orientation.w = 1.0;
        ref_path_msg.poses.push_back(pose);
    }
    mpc_reference_path_pub_->publish(ref_path_msg);

    // Publish open-loop paths if active
    if (open_loop_active_) {
        geometry_msgs::msg::PoseStamped actual_pose;
        actual_pose.header.frame_id = "world";
        actual_pose.header.stamp = current_time;
        actual_pose.pose.position.x = drone_x;
        actual_pose.pose.position.y = drone_y;
        actual_pose.pose.position.z = drone_z;
        actual_pose.pose.orientation.x = drone_qx;
        actual_pose.pose.orientation.y = drone_qy;
        actual_pose.pose.orientation.z = drone_qz;
        actual_pose.pose.orientation.w = drone_qw;
        open_loop_actual_path_.poses.push_back(actual_pose);
        open_loop_actual_path_.header.stamp = current_time;

        // Update timestamps on the predicted path for Foxglove
        open_loop_predicted_path_.header.stamp = current_time;
        for (auto& pose : open_loop_predicted_path_.poses) {
            pose.header.stamp = current_time;
        }

        open_loop_predicted_path_pub_->publish(open_loop_predicted_path_);
        open_loop_actual_path_pub_->publish(open_loop_actual_path_);
    }
}

double UavMpcNode::hoverThrottle() {
    return this->get_parameter("use_px4_hover_estimate").as_bool() ? px4_hover_thrust_
                                                                    : this->get_parameter("hover_throttle").as_double();
}

void UavMpcNode::publishOffboardControlMode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position = false;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = true;
    msg.body_rate = false;
    msg.thrust_and_torque = false;
    msg.direct_actuator = false;
    offboard_control_mode_pub_->publish(msg);
}

void UavMpcNode::publishAttitudeSetpoint(const UavControlOutput& output) {
    px4_msgs::msg::VehicleAttitudeSetpoint att_msg{};
    att_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    
    double q_d[4];
    uav_mpc::kinematics::computeDesiredQuaternion(output.u_opt[0], output.u_opt[1], output.psi_cmd, q_d);
    att_msg.q_d[0] = static_cast<float>(q_d[0]);
    att_msg.q_d[1] = static_cast<float>(q_d[1]);
    att_msg.q_d[2] = static_cast<float>(q_d[2]);
    att_msg.q_d[3] = static_cast<float>(q_d[3]);
    
    att_msg.thrust_body[0] = 0.0f;
    att_msg.thrust_body[1] = 0.0f;
    
    double thrust_normalized = kinematics::thrustToThrottle(
        output.u_opt[2], this->get_parameter("mass").as_double(), this->get_parameter("gravity").as_double(), hoverThrottle(),
        this->get_parameter("rotor_speed_min").as_double(), this->get_parameter("rotor_speed_max").as_double());
    att_msg.thrust_body[2] = -static_cast<float>(thrust_normalized); // -Z in FRD frame is upward force

    attitude_setpoint_pub_->publish(att_msg);
}

void UavMpcNode::publishVisualizationMarkers(const UavControlOutput& output) {
    auto current_time = this->get_clock()->now();

    auto line_msg = telemetry::createPredictedTrajectoryMarker(output.predicted_positions, current_time);
    predicted_trajectory_pub_->publish(line_msg);

    if (output.current_reference.size() == 3) {
        auto point_msg = telemetry::createTargetPointMarker(output.current_reference, current_time);
        target_point_pub_->publish(point_msg);
    }

    if (!output.reference_path.empty()) {
        auto ref_path_msg = telemetry::createReferenceTrajectoryMarker(output.reference_path, current_time);
        reference_trajectory_pub_->publish(ref_path_msg);
    }
}

} // namespace uav_mpc

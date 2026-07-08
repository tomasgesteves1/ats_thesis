#include "uav_mpc/uav_mpc_node.hpp"
#include "uav_mpc/uav_mpc_kinematics.hpp"
#include <limits>
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
      px4_hover_thrust_(0.7265),
      current_system_id_(2),
      last_boat_horizon_time_(0, 0, RCL_ROS_TIME),
      active_tether_max_length_(-1.0),
      open_loop_test_(false) {
      
    pipeline_ = std::make_unique<UavMpcPipeline>();

    // Declare dynamic parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<bool>("open_loop_test", false);
    this->declare_parameter<bool>("use_tether", true);
    this->declare_parameter<double>("tether_rate_limit", 1.5);
    rcl_interfaces::msg::ParameterDescriptor tether_desc;
    tether_desc.dynamic_typing = true;
    this->declare_parameter("tether_max_length", rclcpp::ParameterValue(15.0), tether_desc);
    this->declare_parameter<std::string>("trajectory_type", "hold");
    this->declare_parameter<double>("circle_radius", 3.0);
    this->declare_parameter<double>("circle_omega", 0.2);
    this->declare_parameter<double>("circle_height", 10.0);
    this->declare_parameter<double>("v_max", 10.0);
    this->declare_parameter<double>("u_max", 19.62);
    this->declare_parameter<double>("hover_throttle", 0.7265);
    this->declare_parameter<double>("tilt_max", 0.4);
    this->declare_parameter<double>("hold_height", 2.0);
    this->declare_parameter<double>("takeoff_height", 4.0);
    this->declare_parameter<double>("weight_position", 20.0);
    this->declare_parameter<double>("weight_velocity", 5.0);

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
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", qos);

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

    virtual_tether_limit_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "virtual_tether_limit", 10);

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
}

void UavMpcNode::controlLoop() {
    open_loop_test_ = this->get_parameter("open_loop_test").as_bool();
    pipeline_->setOpenLoopMode(open_loop_test_);

    bool use_tether = this->get_parameter("use_tether").as_bool();
    pipeline_->setUseTether(use_tether);

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
            pipeline_->updateAnchorPosition(
                anchor_x,
                anchor_y,
                transform.transform.translation.z
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
        tf_success = true;

        // Compute and publish virtual tether distance and limit for plotting/monitoring
        if (use_tether) {
            double dx = drone_x - anchor_x;
            double dy = drone_y - anchor_y;
            double dz = drone_z; // Boat Z is assumed 0 in XY projection constraint
            double virtual_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

            std_msgs::msg::Float64 dist_msg;
            dist_msg.data = virtual_dist;
            virtual_tether_distance_pub_->publish(dist_msg);

            std_msgs::msg::Float64 limit_msg;
            limit_msg.data = active_tether_max_length_;
            virtual_tether_limit_pub_->publish(limit_msg);
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
    if (traj_type == "circle") {
        pipeline_->setTrajectoryType(TrajectoryType::CIRCLE);
    } else if (traj_type == "external") {
        pipeline_->setTrajectoryType(TrajectoryType::EXTERNAL);
    } else {
        pipeline_->setTrajectoryType(TrajectoryType::HOLD);
    }

    double radius = this->get_parameter("circle_radius").as_double();
    double omega = this->get_parameter("circle_omega").as_double();
    double height = this->get_parameter("circle_height").as_double();
    pipeline_->configureCircle(radius, omega, height);

    // Update dynamic limits
    double v_max = this->get_parameter("v_max").as_double();
    double u_max = this->get_parameter("u_max").as_double();
    double hover_throttle = this->get_parameter("hover_throttle").as_double();
    double tilt_max = this->get_parameter("tilt_max").as_double();

    pipeline_->setVelocityLimit(v_max);
    pipeline_->setInputLimit(u_max);
    pipeline_->setHoverThrottle(hover_throttle);
    pipeline_->setTiltMax(tilt_max);

    // Update cost weights dynamically
    double w_pos = this->get_parameter("weight_position").as_double();
    double w_vel = this->get_parameter("weight_velocity").as_double();
    pipeline_->setCostWeights(w_pos, w_vel);

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
        if (open_loop_test_ && !pipeline_->isOpenLoopActive()) {
            RCLCPP_INFO(this->get_logger(), "Transition to OFFBOARD detected with open-loop test enabled. Capturing horizon...");
            pipeline_->captureOpenLoopHorizon(sim_time_s);
            
            // Build and store the static open-loop predicted path
            auto current_time_ros = this->get_clock()->now();
            open_loop_predicted_path_.header.frame_id = "world";
            open_loop_predicted_path_.header.stamp = current_time_ros;
            open_loop_predicted_path_.poses.clear();
            const auto& pred_positions = pipeline_->getCapturedPredictedPositions();
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
            
            // Clear the actual path followed
            open_loop_actual_path_.header.frame_id = "world";
            open_loop_actual_path_.header.stamp = current_time_ros;
            open_loop_actual_path_.poses.clear();
        }
    } else {
        pipeline_->resetOpenLoop();
    }

    // Execute calculations in the logic pipeline (pass actual sim time)
    UavControlOutput output = pipeline_->computeControl(sim_time_s);

    // Calculate dynamically normalized thrust using the real-time PX4 hover thrust estimate (throttled to 1Hz)
    double thrust_normalized = (output.u_opt[2] / 9.81) * px4_hover_thrust_;
    thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));

    // Calculate current Euler angles
    double drone_roll = 0.0, drone_pitch = 0.0, drone_yaw = 0.0;
    kinematics::quaternionToEuler(drone_qx, drone_qy, drone_qz, drone_qw, drone_roll, drone_pitch, drone_yaw);

    // Print MPC optimal commands and current attitude throttled to 1Hz
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[MPC Output] Cmd - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f° | Thrust Accel: %.2f m/s^2",
        output.u_opt[0] * 180.0 / M_PI,
        output.u_opt[1] * 180.0 / M_PI,
        drone_yaw * 180.0 / M_PI,
        output.u_opt[2]);

    // Always publish OffboardControlMode to feed PX4 watchdog and allow offboard transition
    publishOffboardControlMode();

    // Always publish VehicleAttitudeSetpoint to allow offboard transition
    publishAttitudeSetpoint(output);

    // Publish RViz/Foxglove visualization markers
    publishVisualizationMarkers(output);

    // Publish estimated tether force magnitude
    std_msgs::msg::Float64 force_msg;
    force_msg.data = output.mpc_tether_force_mag;
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
        output.u_opt[2],                   // 8: Input thrust accel cmd (m/s^2)
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
        output.current_reference_velocity[2]  // 20: Reference vz world (m/s)
    };
    
    // Add layout descriptors for readability
    std_msgs::msg::MultiArrayDimension dim;
    dim.label = "x,y,z,vx,vy,vz,roll_cmd,pitch_cmd,thrust_cmd,roll,pitch,yaw,ref_x,ref_y,ref_z,vx_world,vy_world,vz_world,ref_vx,ref_vy,ref_vz";
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
    if (pipeline_->isOpenLoopActive()) {
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
    uav_mpc::kinematics::computeDesiredQuaternion(output.u_opt[0], output.u_opt[1], 0.0, q_d);
    att_msg.q_d[0] = static_cast<float>(q_d[0]);
    att_msg.q_d[1] = static_cast<float>(q_d[1]);
    att_msg.q_d[2] = static_cast<float>(q_d[2]);
    att_msg.q_d[3] = static_cast<float>(q_d[3]);
    
    att_msg.thrust_body[0] = 0.0f;
    att_msg.thrust_body[1] = 0.0f;
    
    // Normalize thrust using the real-time PX4 hover thrust estimate
    double thrust_normalized = (output.u_opt[2] / 9.81) * px4_hover_thrust_;
    thrust_normalized = std::max(0.0, std::min(thrust_normalized, 1.0));
    att_msg.thrust_body[2] = -static_cast<float>(thrust_normalized); // -Z in FRD frame is upward force

    attitude_setpoint_pub_->publish(att_msg);
}

void UavMpcNode::publishVehicleCommand(uint16_t command, float param1, float param2, float param7) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    // Set param5 and param6 to NaN to ensure local takeoff (avoid Null Island bug)
    msg.param5 = std::numeric_limits<float>::quiet_NaN();
    msg.param6 = std::numeric_limits<float>::quiet_NaN();
    msg.param7 = param7;
    msg.target_system = current_system_id_;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
}

void UavMpcNode::publishVisualizationMarkers(const UavControlOutput& output) {
    auto current_time = this->get_clock()->now();

    // 1. Predicted Trajectory Marker (MPC Horizon)
    visualization_msgs::msg::Marker line_msg;
    line_msg.header.frame_id = "world";
    line_msg.header.stamp = current_time;
    line_msg.ns = "predicted_trajectory";
    line_msg.id = 0;
    line_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_msg.action = visualization_msgs::msg::Marker::ADD;
    line_msg.pose.orientation.w = 1.0;
    
    // Visual settings (semi-transparent bright green)
    line_msg.scale.x = 0.05; // Line width
    line_msg.color.r = 0.0;
    line_msg.color.g = 1.0;
    line_msg.color.b = 0.0;
    line_msg.color.a = 0.8;

    for (const auto& pos : output.predicted_positions) {
        geometry_msgs::msg::Point p;
        p.x = pos[0];
        p.y = pos[1];
        p.z = pos[2];
        line_msg.points.push_back(p);
    }
    predicted_trajectory_pub_->publish(line_msg);

    // 2. Current Reference/Target Marker
    if (output.current_reference.size() == 3) {
        visualization_msgs::msg::Marker point_msg;
        point_msg.header.frame_id = "world";
        point_msg.header.stamp = current_time;
        point_msg.ns = "target_point";
        point_msg.id = 1;
        point_msg.type = visualization_msgs::msg::Marker::SPHERE;
        point_msg.action = visualization_msgs::msg::Marker::ADD;
        
        point_msg.pose.position.x = output.current_reference[0];
        point_msg.pose.position.y = output.current_reference[1];
        point_msg.pose.position.z = output.current_reference[2];
        point_msg.pose.orientation.w = 1.0;

        point_msg.scale.x = 0.25; // 25 cm diameter
        point_msg.scale.y = 0.25;
        point_msg.scale.z = 0.25;

        point_msg.color.r = 1.0; // Red
        point_msg.color.g = 0.0;
        point_msg.color.b = 0.0;
        point_msg.color.a = 1.0;

        target_point_pub_->publish(point_msg);
    }

    // 3. Complete Reference Trajectory Marker (e.g. circle in blue)
    if (!output.reference_path.empty()) {
        visualization_msgs::msg::Marker ref_path_msg;
        ref_path_msg.header.frame_id = "world";
        ref_path_msg.header.stamp = current_time;
        ref_path_msg.ns = "reference_trajectory";
        ref_path_msg.id = 2;
        ref_path_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
        ref_path_msg.action = visualization_msgs::msg::Marker::ADD;
        ref_path_msg.pose.orientation.w = 1.0;

        // Visual settings (bright blue)
        ref_path_msg.scale.x = 0.03; // Line thickness
        ref_path_msg.color.r = 0.0;
        ref_path_msg.color.g = 0.5;
        ref_path_msg.color.b = 1.0;
        ref_path_msg.color.a = 0.9;

        for (const auto& pos : output.reference_path) {
            geometry_msgs::msg::Point p;
            p.x = pos[0];
            p.y = pos[1];
            p.z = pos[2];
            ref_path_msg.points.push_back(p);
        }
        reference_trajectory_pub_->publish(ref_path_msg);
    }
}

} // namespace uav_mpc

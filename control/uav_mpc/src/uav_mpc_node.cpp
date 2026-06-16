#include "uav_mpc/uav_mpc_node.hpp"

namespace uav_mpc {

UavMpcNode::UavMpcNode() 
    : Node("uav_mpc_node"), 
      target_initialized_(false), 
      offboard_setpoint_counter_(0) {
      
    pipeline_ = std::make_unique<UavMpcPipeline>();

    // Declare dynamic parameters (Rule 2 of CODE_STANDARDS.md)
    this->declare_parameter<bool>("use_tether", true);
    this->declare_parameter<std::string>("trajectory_type", "hold");
    this->declare_parameter<double>("circle_radius", 3.0);
    this->declare_parameter<double>("circle_omega", 0.2);
    this->declare_parameter<double>("circle_height", 10.0);
    this->declare_parameter<double>("v_max", 10.0);
    this->declare_parameter<double>("u_max", 19.62);
    this->declare_parameter<double>("hover_throttle", 0.52);
    this->declare_parameter<double>("tilt_max", 0.4);
    this->declare_parameter<double>("hold_height", 2.0);

    // Relative namespaces for topics (Rule 4 of CODE_STANDARDS.md)
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&UavMpcNode::odomCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "target_position", 10, std::bind(&UavMpcNode::targetCallback, this, std::placeholders::_1));

    tether_length_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "tether_length", 10, std::bind(&UavMpcNode::tetherLengthCallback, this, std::placeholders::_1));

    // Initialize TF buffer and listener
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Configure QoS identical to drone_tracker for PX4 (SensorDataQoS)
    auto qos = rclcpp::SensorDataQoS();
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

    // Timer at 20Hz (0.05s) to match the MPC dt
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&UavMpcNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "UAV MPC Node successfully initialized.");
}

void UavMpcNode::targetCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "New reference received: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    pipeline_->setReference({msg->x, msg->y, msg->z});
}

void UavMpcNode::tetherLengthCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    pipeline_->updateTetherLength(msg->data);
}

void UavMpcNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Pure data passing, no maths here (Rule 1 of CODE_STANDARDS.md)
    std::vector<double> state = {
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    };
    pipeline_->updateState(state);
    
    // Pass current orientation to the pipeline
    pipeline_->updateOrientation(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    if (!target_initialized_) {
        double hold_height = this->get_parameter("hold_height").as_double();
        pipeline_->setReference({msg->pose.pose.position.x, msg->pose.pose.position.y, hold_height});
        target_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial target set from odometry: X=%.2f, Y=%.2f, Z=%.2f", 
                    msg->pose.pose.position.x, msg->pose.pose.position.y, hold_height);
    }
}

void UavMpcNode::controlLoop() {
    bool use_tether = this->get_parameter("use_tether").as_bool();
    pipeline_->setUseTether(use_tether);

    if (use_tether) {
        // Look up the exact anchor position using TF (world -> boat/tether_anchor)
        try {
            auto transform = tf_buffer_->lookupTransform("world", "boat/tether_anchor", tf2::TimePointZero);
            pipeline_->updateAnchorPosition(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            );
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Could not obtain transform from world to boat/tether_anchor: %s", ex.what());
        }
    } else {
        // If disabled, keep anchor at origin
        pipeline_->updateAnchorPosition(0.0, 0.0, 0.0);
    }

    // Update dynamic trajectory parameters
    std::string traj_type = this->get_parameter("trajectory_type").as_string();
    if (traj_type == "circle") {
        pipeline_->setTrajectoryType(TrajectoryType::CIRCLE);
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

    // Execute calculations in the logic pipeline
    UavControlOutput output = pipeline_->computeControl();

    // Log the raw MPC control inputs and their conversion to degrees (throttled to 1Hz)
    double roll_deg = output.u_opt[0] * 180.0 / 3.14159265358979323846;
    double pitch_deg = output.u_opt[1] * 180.0 / 3.14159265358979323846;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "MPC Raw -> Roll: %.4f rad (%.2f deg) | Pitch: %.4f rad (%.2f deg) | Thrust: %.2f m/s^2",
        output.u_opt[0], roll_deg, output.u_opt[1], pitch_deg, output.u_opt[2]);

    // Always publish OffboardControlMode and VehicleAttitudeSetpoint to feed PX4 watchdog
    // publishOffboardControlMode();
    // publishAttitudeSetpoint(output);
    
    // Publish RViz/Foxglove visualization markers
    publishVisualizationMarkers(output);

    // Publish estimated tether force magnitude
    std_msgs::msg::Float64 force_msg;
    force_msg.data = output.mpc_tether_force_mag;
    mpc_tether_force_pub_->publish(force_msg);

    // Arm and set mode after ~2 seconds (40 iterations at 20Hz)
    /*
    if (offboard_setpoint_counter_ >= 40 && offboard_setpoint_counter_ < 60) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0); // 1 = offboard, 6 = offboard submode
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); // 1.0 = arm
        
        if (offboard_setpoint_counter_ == 40) {
            RCLCPP_INFO(this->get_logger(), "Activating Offboard Mode and Arming Drone on PX4...");
        }
    }
    */

    if (offboard_setpoint_counter_ < 60) {
        offboard_setpoint_counter_++;
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
    
    att_msg.q_d[0] = output.q_d[0];
    att_msg.q_d[1] = output.q_d[1];
    att_msg.q_d[2] = output.q_d[2];
    att_msg.q_d[3] = output.q_d[3];
    
    att_msg.thrust_body[0] = 0.0f;
    att_msg.thrust_body[1] = 0.0f;
    att_msg.thrust_body[2] = -static_cast<float>(output.thrust_normalized); // -Z in FRD frame is upward force

    attitude_setpoint_pub_->publish(att_msg);
}

void UavMpcNode::publishVehicleCommand(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 2; // Drone x500
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

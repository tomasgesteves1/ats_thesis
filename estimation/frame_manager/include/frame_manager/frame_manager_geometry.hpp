#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace frame_manager {
namespace geometry {

    /**
     * @brief Converts Odometry to Transform with an optional Z offset.
     */
    geometry_msgs::msg::TransformStamped odometry_to_transform(
        const nav_msgs::msg::Odometry::SharedPtr& odom,
        const std::string& frame_id,
        const std::string& child_frame_id,
        double z_offset = 0.0);

} // namespace geometry
} // namespace frame_manager

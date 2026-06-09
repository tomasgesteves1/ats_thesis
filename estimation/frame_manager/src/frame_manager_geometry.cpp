#include "frame_manager/frame_manager_geometry.hpp"

namespace frame_manager {
namespace geometry {

geometry_msgs::msg::TransformStamped odometry_to_transform(
    const nav_msgs::msg::Odometry::SharedPtr& odom,
    const std::string& frame_id,
    const std::string& child_frame_id,
    double z_offset)
{
    geometry_msgs::msg::TransformStamped t;

    if (!odom) {
        return t;
    }

    t.header.stamp = odom->header.stamp;
    t.header.frame_id = frame_id;
    t.child_frame_id = child_frame_id;

    // Apply translation with vertical offset
    t.transform.translation.x = odom->pose.pose.position.x;
    t.transform.translation.y = odom->pose.pose.position.y;
    t.transform.translation.z = odom->pose.pose.position.z + z_offset;

    // Copy orientation (assuming ENU for now)
    t.transform.rotation = odom->pose.pose.orientation;

    return t;
}

} // namespace geometry
} // namespace frame_manager

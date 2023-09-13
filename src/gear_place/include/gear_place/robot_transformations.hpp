#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/logger.hpp>

namespace robot_transformations
{
  geometry_msgs::msg::Pose MultiplyPose(geometry_msgs::msg::Pose p1, geometry_msgs::msg::Pose p2)
  {
    KDL::Frame f1;
    KDL::Frame f2;

    tf2::fromMsg(p1, f1);
    tf2::fromMsg(p2, f2);

    KDL::Frame f3 = f1 * f2;

    return tf2::toMsg(f3);
  }

  geometry_msgs::msg::Pose BuildPose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = orientation;

    return pose;
  }

  geometry_msgs::msg::Pose PoseFromTransform(geometry_msgs::msg::Transform transform)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;

    return pose;
  }

  double GetYawFromPose(geometry_msgs::msg::Pose pose)
  {
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }

  geometry_msgs::msg::Quaternion QuaternionFromRPY(double r, double p, double y)
  {
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(r, p, y);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
  }

  void log_pose(rclcpp::Logger l, geometry_msgs::msg::Pose p)
  {
    RCLCPP_INFO_STREAM(l,
                       "Position: {x: " << std::to_string(p.position.x) << " "
                                        << "y: " << std::to_string(p.position.y) << " "
                                        << "z: " << std::to_string(p.position.z) << " } "
                                        << "Orientation: x: " << std::to_string(p.orientation.x) << " "
                                        << "y: " << std::to_string(p.orientation.y) << " "
                                        << "z: " << std::to_string(p.orientation.z) << " "
                                        << "w: " << std::to_string(p.orientation.w) << " ");
  }
}
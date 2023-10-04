#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <gear_place/robot_transformations.hpp>
#include <gear_place/motion_generators.hpp>

#include <gear_place_interfaces/srv/move_cartesian.hpp>
#include <gear_place_interfaces/srv/move_to_named_pose.hpp>
#include <gear_place_interfaces/srv/pick_up_gear.hpp>
#include <gear_place_interfaces/srv/move_to_conveyor.hpp>
#include <gear_place_interfaces/srv/put_gear_down.hpp>
#include <gear_place_interfaces/srv/pick_up_moving_gear.hpp>
#include <gear_place_interfaces/srv/open_gripper.hpp>
#include <gear_place_interfaces/srv/put_down_force.hpp>
#include <gear_place_interfaces/srv/get_camera_angle.hpp>
#include <gear_place_interfaces/srv/move_cartesian_angle.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/exception.h>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"

#include <cmath>
#include <Eigen/Core>
#include <mutex>
#include <ctime>
#include <chrono>


class CommanderError : public std::exception
{
public:
  std::string message_;
  CommanderError(std::string message) : message_(message){};
  std::string what()
  {
    return message_;
  }
};

class RobotCommander : public rclcpp::Node
{
public:
  RobotCommander(const std::string &);

private:
  // RCLCPP
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr camera_angle_pub_;

  rclcpp::Service<gear_place_interfaces::srv::MoveToNamedPose>::SharedPtr move_to_named_pose_srv_;
  rclcpp::Service<gear_place_interfaces::srv::MoveCartesian>::SharedPtr move_cartesian_srv_;
  rclcpp::Service<gear_place_interfaces::srv::PickUpGear>::SharedPtr pick_up_gear_srv_;
  rclcpp::Service<gear_place_interfaces::srv::PutGearDown>::SharedPtr put_gear_down_srv_;
  rclcpp::Service<gear_place_interfaces::srv::PickUpMovingGear>::SharedPtr pick_up_moving_gear_srv_;
  rclcpp::Service<gear_place_interfaces::srv::OpenGripper>::SharedPtr open_gripper_srv_;
  rclcpp::Service<gear_place_interfaces::srv::PutDownForce>::SharedPtr put_down_force_srv_;
  rclcpp::Service<gear_place_interfaces::srv::GetCameraAngle>::SharedPtr get_camera_angle_srv_;
  rclcpp::Service<gear_place_interfaces::srv::MoveCartesianAngle>::SharedPtr move_cartesian_angle_srv_;

  rclcpp::CallbackGroup::SharedPtr publisher_cb_group_;

  rclcpp::TimerBase::SharedPtr joint_state_publish_timer_;
  rclcpp::TimerBase::SharedPtr ee_pose_publish_timer_;
  rclcpp::TimerBase::SharedPtr camera_angle_publish_timer_;
  // Franka
  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Gripper> gripper_;
  franka::RobotState current_state_;
  franka::GripperState gripper_state_;
  std::unique_ptr<franka::Model> robot_model_;

  // KDL
  std::string robot_description_;
  KDL::Tree kdl_tree_;
  KDL::Chain chain_;
  int num_joints_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;

  // Mutex
  std::mutex read_state_;

  // Joint State
  sensor_msgs::msg::JointState joint_state_msg_;
  std::vector<std::string> joint_names_{"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5",
                                        "fr3_joint6", "fr3_joint7", "fr3_finger_joint1", "fr3_finger_joint2"};

  // EE Pose
  geometry_msgs::msg::PoseStamped ee_pose_;

  // Camera Angle
  std_msgs::msg::Float64 camera_angle_;

  // Callbacks
  void joint_state_publish_timer_cb_();
  void ee_pose_publish_timer_cb_();
  void camera_angle_publish_timer_cb_();

  void move_to_named_pose_cb_(const std::shared_ptr<gear_place_interfaces::srv::MoveToNamedPose::Request> request,
                              std::shared_ptr<gear_place_interfaces::srv::MoveToNamedPose::Response> response);
  void move_cartesian_cb_(const std::shared_ptr<gear_place_interfaces::srv::MoveCartesian::Request> request,
                          std::shared_ptr<gear_place_interfaces::srv::MoveCartesian::Response> response);
  void pick_up_gear_cb_(const std::shared_ptr<gear_place_interfaces::srv::PickUpGear::Request> request,
                        std::shared_ptr<gear_place_interfaces::srv::PickUpGear::Response> response);
  void put_gear_down_cb_(const std::shared_ptr<gear_place_interfaces::srv::PutGearDown::Request> request,
                         std::shared_ptr<gear_place_interfaces::srv::PutGearDown::Response> response);
  void pick_up_moving_gear_cb_(const std::shared_ptr<gear_place_interfaces::srv::PickUpMovingGear::Request> request,
                               std::shared_ptr<gear_place_interfaces::srv::PickUpMovingGear::Response> response);
  void open_gripper_cb_(const std::shared_ptr<gear_place_interfaces::srv::OpenGripper::Request> request,
                        std::shared_ptr<gear_place_interfaces::srv::OpenGripper::Response> response);
  void put_down_force_cb_(const std::shared_ptr<gear_place_interfaces::srv::PutDownForce::Request> request,
                        std::shared_ptr<gear_place_interfaces::srv::PutDownForce::Response> response);
  void get_camera_angle_cb_(const std::shared_ptr<gear_place_interfaces::srv::GetCameraAngle::Request>request,
                          std::shared_ptr<gear_place_interfaces::srv::GetCameraAngle::Response> response);
  void move_cartesian_angle_cb_(const std::shared_ptr<gear_place_interfaces::srv::MoveCartesianAngle::Request>request,
                          std::shared_ptr<gear_place_interfaces::srv::MoveCartesianAngle::Response> response);

  // Methods
  void move_robot_to_frame(KDL::Frame);
  void move_robot_cartesian(double, double, double, double, double);
  void move_robot_cartesian_angle(double, double, double, double, double, double);
  void put_down_force(double);
  void open_gripper();
  void grasp_object(double);
  void set_default_behavior();
  geometry_msgs::msg::Pose solve_fk();
  double get_camera_angle();

  // Constants
  std::map<std::string, std::array<double, 7>> named_joint_positions_ = {
      {"home",{{0.0, 0.196578, 0.0, -1.9388, 0.0, 2.15806, 0.816665}}},
      {"above_conveyor",{{M_PI_2-0.01,-0.115237,0.0,-2.015795,-0.01282,1.91310556,0.816665}}},
      {"position_1", {{0, 0, 0, -M_PI_2, 0, M_PI_2, 0.7}}},
      {"position_2", {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}}}
  };
  const double gripper_speed_ = 0.1;
  const double gripper_force_ = 10;
  const double default_velocity_ = 0.15;
  const double default_acceleration_ = 0.2;
  const double wait_time_ = 0.25;
};
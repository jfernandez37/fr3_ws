#include <rclcpp/rclcpp.hpp>

#include <gmcs_panda_task_board/robot_transformations.hpp>
#include <gmcs_panda_task_board/motion_generators.hpp>

#include <gmcs_interfaces/srv/move_to_position.hpp>
#include <gmcs_interfaces/srv/move_to_named_pose.hpp>
#include <gmcs_interfaces/srv/pick_object.hpp>
#include <gmcs_interfaces/srv/move_cartesian.hpp>
#include <gmcs_interfaces/srv/move_object_to_board.hpp>
#include <gmcs_interfaces/srv/assemble_object.hpp>

#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/exception.h>

#include <tf2_kdl/tf2_kdl.hpp>

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

  rclcpp::Service<gmcs_interfaces::srv::MoveToPosition>::SharedPtr move_to_position_srv_;
  rclcpp::Service<gmcs_interfaces::srv::MoveToNamedPose>::SharedPtr move_to_named_pose_srv_;
  rclcpp::Service<gmcs_interfaces::srv::PickObject>::SharedPtr pick_object_srv_;
  rclcpp::Service<gmcs_interfaces::srv::MoveCartesian>::SharedPtr move_cartesian_srv_;
  rclcpp::Service<gmcs_interfaces::srv::MoveObjectToBoard>::SharedPtr move_object_to_board_srv_;
  rclcpp::Service<gmcs_interfaces::srv::AssembleObject>::SharedPtr assemble_object_srv_;

  rclcpp::CallbackGroup::SharedPtr publisher_cb_group_;

  rclcpp::TimerBase::SharedPtr joint_state_publish_timer_;
  rclcpp::TimerBase::SharedPtr ee_pose_publish_timer_;

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
  std::vector<std::string> joint_names_{"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
                                        "panda_joint6", "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};

  // EE Pose
  geometry_msgs::msg::PoseStamped ee_pose_;

  // Callbacks
  void joint_state_publish_timer_cb_();
  void ee_pose_publish_timer_cb_();

  void move_to_position_cb_(const std::shared_ptr<gmcs_interfaces::srv::MoveToPosition::Request> request,
                            std::shared_ptr<gmcs_interfaces::srv::MoveToPosition::Response> response);
  void move_to_named_pose_cb_(const std::shared_ptr<gmcs_interfaces::srv::MoveToNamedPose::Request> request,
                              std::shared_ptr<gmcs_interfaces::srv::MoveToNamedPose::Response> response);
  void pick_object_cb_(const std::shared_ptr<gmcs_interfaces::srv::PickObject::Request> request,
                       std::shared_ptr<gmcs_interfaces::srv::PickObject::Response> response);
  void move_cartesian_cb_(const std::shared_ptr<gmcs_interfaces::srv::MoveCartesian::Request> request,
                          std::shared_ptr<gmcs_interfaces::srv::MoveCartesian::Response> response);
  void move_object_to_board_cb_(const std::shared_ptr<gmcs_interfaces::srv::MoveObjectToBoard::Request> request,
                                std::shared_ptr<gmcs_interfaces::srv::MoveObjectToBoard::Response> response);
  void assemble_object_cb_(const std::shared_ptr<gmcs_interfaces::srv::AssembleObject::Request> request,
                           std::shared_ptr<gmcs_interfaces::srv::AssembleObject::Response> response);

  // Methods
  void move_robot_to_frame(KDL::Frame);
  void move_robot_cartesian(double, double, double, double, double);
  bool attempt_install(double, double);
  void open_gripper();
  void grasp_object(double);
  bool random_search(int object_type, double search_radius, int iterations);
  void set_default_behavior();
  geometry_msgs::msg::Pose solve_fk();

  // Helper Function
  KDL::Frame kdl_frame_from_franka_transform(std::array<double, 16UL> transform);

  // Constants
  std::map<std::string, std::array<double, 7>> named_joint_positions_ = {
      // {"home", {{0, 0.2, 0, -M_PI_2, 0, M_PI_2, M_PI_4}}},
      {"home", {{0.0182304, 0.196578, 0.0652365, -1.9388, -0.013601, 2.15806, 0.816665}}},
      {"position_1", {{0, 0, 0, -M_PI_2, 0, M_PI_2, 0.7}}},
  };
  const double gripper_speed_ = 0.1;
  const double gripper_force_ = 10;
  const double default_velocity_ = 0.15;
  const double default_acceleration_ = 0.2;
};
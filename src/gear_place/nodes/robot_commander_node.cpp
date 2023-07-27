#include <gear_place/robot_commander.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto robot_commander = std::make_shared<RobotCommander>("172.16.0.2");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(robot_commander);
  
  executor.spin();

  rclcpp::shutdown();
}
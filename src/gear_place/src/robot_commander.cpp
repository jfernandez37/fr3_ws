#include <gear_place/robot_commander.hpp>

RobotCommander::RobotCommander(const std::string &robot_ip)
    : Node("robot_commander")
{
  // Connect to robot
  try
  {
    robot_ = std::make_unique<franka::Robot>(robot_ip);
    robot_model_ = std::make_unique<franka::Model>(robot_->loadModel());

    set_default_behavior();
    RCLCPP_INFO(get_logger(), "Connected to robot");

    gripper_ = std::make_unique<franka::Gripper>(robot_ip);
    RCLCPP_INFO(get_logger(), "Connected to gripper");

    gripper_state_ = gripper_->readOnce();
  }
  catch (franka::Exception const &e)
  {
    std::cout << e.what() << std::endl;
    throw e;
  }

  // Crate ik_solver
  this->declare_parameter<std::string>("robot_description", "");
  this->get_parameter("robot_description", robot_description_);

  if (!kdl_parser::treeFromString(robot_description_, kdl_tree_))
    RCLCPP_ERROR(get_logger(), "Unable to create KDL tree from URDF");

  if (!kdl_tree_.getChain("world", "panda_hand_tcp", chain_))
    RCLCPP_ERROR(get_logger(), "Unable to create KDL chain from URDF");

  num_joints_ = chain_.getNrOfJoints();

  RCLCPP_INFO_STREAM(get_logger(), "Number of joints: " << std::to_string(num_joints_));

  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain_);
  ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR>(chain_, *fk_pos_solver_, *ik_vel_solver_);

  // ROS Publishers
  publisher_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  ee_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ee_pose", 10);

  // ROS Timers
  joint_state_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&RobotCommander::joint_state_publish_timer_cb_, this),
      publisher_cb_group_);

  ee_pose_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&RobotCommander::ee_pose_publish_timer_cb_, this),
      publisher_cb_group_);

  // ROS Services
  move_to_named_pose_srv_ = this->create_service<gmcs_interfaces::srv::MoveToNamedPose>(
      "move_to_named_pose",
      std::bind(
          &RobotCommander::move_to_named_pose_cb_, this,
          std::placeholders::_1, std::placeholders::_2));

  move_cartesian_srv_ = this->create_service<gmcs_interfaces::srv::MoveCartesian>(
      "move_cart",
      std::bind(&RobotCommander::move_cartesian_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  std::srand(std::time(0)); // use current time as seed for random generator
}


void RobotCommander::move_to_named_pose_cb_(
    const std::shared_ptr<gmcs_interfaces::srv::MoveToNamedPose::Request> request,
    std::shared_ptr<gmcs_interfaces::srv::MoveToNamedPose::Response> response)
{
  if (named_joint_positions_.find(request->pose) != named_joint_positions_.end())
  {
    try
    {
      MotionGenerator motion_generator(0.2, named_joint_positions_[request->pose], current_state_);

      read_state_.lock();
      robot_->control(motion_generator);
      read_state_.unlock();
    }
    catch (const franka::Exception &e)
    {
      response->success = false;
      RCLCPP_WARN_STREAM(get_logger(), "Franka Exception: " << e.what());
      return;
    }

    response->success = true;
  }
  else
  {
    response->success = false;
    RCLCPP_WARN(get_logger(), "Named pose does not exist");
  }
}


void RobotCommander::joint_state_publish_timer_cb_()
{
  joint_state_msg_.name = joint_names_;

  if (read_state_.try_lock())
  {
    current_state_ = robot_->readOnce();
    read_state_.unlock();
  }

  joint_state_msg_.position.clear();
  joint_state_msg_.velocity.clear();
  joint_state_msg_.effort.clear();

  for (int i = 0; i < 7; i++)
  {
    joint_state_msg_.position.push_back(current_state_.q.at(i));
    joint_state_msg_.velocity.push_back(current_state_.dq.at(i));
    joint_state_msg_.effort.push_back(current_state_.tau_J.at(i));
  }

  // Handle finger joints
  joint_state_msg_.position.push_back(0);
  joint_state_msg_.position.push_back(0);

  joint_state_msg_.velocity.push_back(0);
  joint_state_msg_.velocity.push_back(0);

  joint_state_msg_.effort.push_back(0);
  joint_state_msg_.effort.push_back(0);

  joint_state_msg_.header.stamp = this->get_clock()->now();
  joint_state_pub_->publish(joint_state_msg_);
}

void RobotCommander::ee_pose_publish_timer_cb_()
{
  ee_pose_.header.frame_id = "world";
  ee_pose_.header.stamp = this->get_clock()->now();

  ee_pose_.pose = solve_fk();

  ee_pose_pub_->publish(ee_pose_);
}

geometry_msgs::msg::Pose RobotCommander::solve_fk()
{

  KDL::JntArray q_in = KDL::JntArray(num_joints_);

  for (int i = 0; i < num_joints_; i++)
  {
    q_in(i) = current_state_.q.at(i);
  }

  KDL::Frame f;

  fk_pos_solver_->JntToCart(q_in, f);

  return tf2::toMsg(f);
}

void RobotCommander::move_cartesian_cb_(
    const std::shared_ptr<gmcs_interfaces::srv::MoveCartesian::Request> request,
    std::shared_ptr<gmcs_interfaces::srv::MoveCartesian::Response> response)
{
  try
  {
    move_robot_cartesian(request->x, request->y, request->z, request->max_velocity, request->acceleration);
  }
  catch (CommanderError &e)
  {
    std::string err = e.what();
    RCLCPP_ERROR(get_logger(), err.c_str());
    response->success = false;
    return;
  }
  response->success = true;
}

void RobotCommander::move_robot_cartesian(double x, double y, double z, double maximum_velocity, double acceleration)
{
  std::unique_ptr<CartesianMotionGenerator> cartesian_motion_generator;

  try
  {
    cartesian_motion_generator = std::make_unique<CartesianMotionGenerator>(x, y, z, maximum_velocity, acceleration, current_state_);
  }
  catch (InvalidParameters &ip)
  {
    throw CommanderError(ip.what());
  }

  try
  {
    read_state_.lock();
    robot_->control(*cartesian_motion_generator);
    read_state_.unlock();
  }
  catch (const franka::Exception &e)
  {
    std::string ex = e.what();
    throw CommanderError("Franka Exception: " + ex);
  }
}


void RobotCommander::open_gripper()
{
  try
  {
    gripper_->move(gripper_state_.max_width - 0.01, gripper_speed_);
  }
  catch (const franka::Exception &e)
  {
    std::string ex = e.what();
    throw CommanderError("Franka Exception: " + ex);
  }
  RCLCPP_INFO(get_logger(), "Gripper opened");
}

void RobotCommander::grasp_object(double object_width)
{
  if (gripper_state_.max_width < object_width)
  {
    throw CommanderError("Object width is larger than gripper max width");
  }
  if (!gripper_->grasp(object_width, gripper_speed_, gripper_force_))
  {
    throw CommanderError("Unable to grasp object");
  }
}

void RobotCommander::set_default_behavior()
{
  robot_->setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}
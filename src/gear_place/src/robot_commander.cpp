#include <gmcs_panda_task_board/robot_commander.hpp>

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
  move_to_position_srv_ = this->create_service<gmcs_interfaces::srv::MoveToPosition>(
      "move_to_position",
      std::bind(
          &RobotCommander::move_to_position_cb_, this,
          std::placeholders::_1, std::placeholders::_2));

  move_to_named_pose_srv_ = this->create_service<gmcs_interfaces::srv::MoveToNamedPose>(
      "move_to_named_pose",
      std::bind(
          &RobotCommander::move_to_named_pose_cb_, this,
          std::placeholders::_1, std::placeholders::_2));

  pick_object_srv_ = this->create_service<gmcs_interfaces::srv::PickObject>(
      "pick_object",
      std::bind(
          &RobotCommander::pick_object_cb_, this,
          std::placeholders::_1, std::placeholders::_2));

  move_cartesian_srv_ = this->create_service<gmcs_interfaces::srv::MoveCartesian>(
      "move_cart",
      std::bind(&RobotCommander::move_cartesian_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  move_object_to_board_srv_ = this->create_service<gmcs_interfaces::srv::MoveObjectToBoard>(
      "move_object_to_board",
      std::bind(&RobotCommander::move_object_to_board_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  assemble_object_srv_ = this->create_service<gmcs_interfaces::srv::AssembleObject>(
      "assemble_object",
      std::bind(&RobotCommander::assemble_object_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  std::srand(std::time(0)); // use current time as seed for random generator
}

void RobotCommander::move_to_position_cb_(
    const std::shared_ptr<gmcs_interfaces::srv::MoveToPosition::Request> request,
    std::shared_ptr<gmcs_interfaces::srv::MoveToPosition::Response> response)
{
  // Create target EE pose from home orientation and target position
  KDL::Frame target_frame(
      KDL::Rotation::RPY(M_PI, 0.0, 0.1),
      KDL::Vector(request->target_position.x, request->target_position.y, request->target_position.z));

  try
  {
    move_robot_to_frame(target_frame);
  }
  catch (CommanderError &e)// use current time as seed for random generator
  {
    std::string err = e.what();
    RCLCPP_ERROR(get_logger(), err.c_str());
    response->success = false;
    return;
  }
  response->success = true;
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

void RobotCommander::pick_object_cb_(
    const std::shared_ptr<gmcs_interfaces::srv::PickObject::Request> request,
    std::shared_ptr<gmcs_interfaces::srv::PickObject::Response> response)
{
  double grip_offset_height = 0.2;
  geometry_msgs::msg::Pose grip_pose = robot_transformations::MultiplyPose(
      request->object_pose,
      robot_transformations::PoseFromTransform(request->object.grasp_transform));

  KDL::Frame above_part_frame(
      KDL::Rotation::RPY(M_PI, 0.0, 0.1),
      KDL::Vector(grip_pose.position.x, grip_pose.position.y, grip_pose.position.z + grip_offset_height));

  try
  {
    move_robot_to_frame(above_part_frame);
    open_gripper();
    move_robot_cartesian(0, 0, -grip_offset_height, default_velocity_, default_acceleration_);
    grasp_object(request->object.grasp_width);
    move_robot_cartesian(0, 0, grip_offset_height, default_velocity_, default_acceleration_);
  }
  catch (CommanderError &e)
  {
    std::string err = e.what();
    RCLCPP_ERROR(get_logger(), err.c_str());
    response->success = false;
    return;
  }

  // Check that object is still grasped
  gripper_state_ = gripper_->readOnce();
  if (!gripper_state_.is_grasped)
  {
    RCLCPP_WARN(get_logger(), "Object was not grasped");
    response->success = false;
  }

  response->success = true;
}

void RobotCommander::move_object_to_board_cb_(
    const std::shared_ptr<gmcs_interfaces::srv::MoveObjectToBoard::Request> request,
    std::shared_ptr<gmcs_interfaces::srv::MoveObjectToBoard::Response> response)
{
  double offset_height = 0.2;
  double surface_offset = 0.005;

  geometry_msgs::msg::Pose assembly_pose = robot_transformations::MultiplyPose(
      request->assembly_pose,
      robot_transformations::PoseFromTransform(request->object.grasp_transform));

  KDL::Frame above_assembly_frame(
      KDL::Rotation::RPY(M_PI, 0.0, 0.1),
      KDL::Vector(assembly_pose.position.x, assembly_pose.position.y, assembly_pose.position.z + offset_height));

  try
  {
    // Move robot to position above asssembly 
    move_robot_to_frame(above_assembly_frame);
    
    // Move down most of the way to the board
    move_robot_cartesian(0, 0, -offset_height + surface_offset, default_velocity_, default_acceleration_);

    // Slowly move down to touch the board
    move_robot_cartesian(0.0, 0.0, -surface_offset, 0.01, 0.05);
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

void RobotCommander::assemble_object_cb_(
    const std::shared_ptr<gmcs_interfaces::srv::AssembleObject::Request> request,
    std::shared_ptr<gmcs_interfaces::srv::AssembleObject::Response> response)
{
  if (request->search_method == gmcs_interfaces::srv::AssembleObject::Request::RANDOM_SEARCH)
  {
    try
    {
      response->success = random_search(request->object.type, 0.01, 10);
      open_gripper();
    }
    catch (CommanderError &e)
    {
      open_gripper();
      std::string err = e.what();
      RCLCPP_ERROR(get_logger(), err.c_str());
      response->success = false;
      return;
    }
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

bool RobotCommander::random_search(int type, double search_radius, int iterations)
{
  geometry_msgs::msg::Pose pose_i = solve_fk(); 
  double x_i = pose_i.position.x;
  double y_i = pose_i.position.x;

  if (type == gmcs_interfaces::msg::AssemblyObject::CIRCLE_PEG)
  {
    for (int i = 0; i < iterations; i++)
    {
      try
      {
        if (attempt_install(6.0, 0.01))
        {
          RCLCPP_INFO_STREAM(get_logger(),"Install successful");
          return true;
        }

        // Generate new point in search radius
        double r = ((double)std::rand() / RAND_MAX) * search_radius;
        double angle = ((double)std::rand() / RAND_MAX) * M_2_PI; 

        double x_t = x_i + r * std::cos(angle);
        double y_t = y_i + r * std::sin(angle);

        // Move to new target in search radius
        geometry_msgs::msg::Pose current_pose = solve_fk();
        RCLCPP_INFO_STREAM(get_logger(), "x target: "<<std::to_string(x_t) << " y target: "<< std::to_string(y_t));
        move_robot_cartesian(x_t - current_pose.position.x, y_t - current_pose.position.y, 0, 0.01, 0.05);

        // RCLCPP_INFO_STREAM(get_logger(), "X value: "<<std::to_string(r * std::cos(angle))<<" Y value: "<<std::to_string(r * std::sin(angle)));
        // RCLCPP_INFO_STREAM(get_logger(), "r value: "<<std::to_string(r)<<" angle value: "<<std::to_string(angle));

      }
      catch (CommanderError &e)
      {
        throw e;
      }
    }
  }

  return false;
}

KDL::Frame RobotCommander::kdl_frame_from_franka_transform(std::array<double, 16UL> transform)
{
  double xx, yx, zx, xy, yy, zy, xz, yz, zz, tx, ty, tz;
  xx = transform[0];
  yx = transform[4];
  zx = transform[8];
  xy = transform[1];
  yy = transform[5];
  zy = transform[9];
  xz = transform[2];
  yz = transform[6];
  zz = transform[10];

  tx = transform[12];
  ty = transform[13];
  tz = transform[14];

  return KDL::Frame(
      KDL::Rotation(xx, yx, zx, xy, yy, zy, xz, yz, zz),
      KDL::Vector(tx, ty, tz));
}

void RobotCommander::move_robot_to_frame(KDL::Frame target_frame)
{
  KDL::JntArray q_init = KDL::JntArray(num_joints_);
  KDL::JntArray q_out = KDL::JntArray(num_joints_);

  for (int i = 0; i < num_joints_; i++)
  {
    q_init(i) = current_state_.q.at(i);
  }

  int ret = ik_pos_solver_->CartToJnt(q_init, target_frame, q_out);

  if (ret != 0)
  {
    std::string err = ik_pos_solver_->strError(ret);
    throw CommanderError("Unable to solve the inverse kinematics. Error: " + err);
  }

  std::array<double, 7> q_goal{{q_out(0), q_out(1), q_out(2), q_out(3), q_out(4), q_out(5), q_out(6)}};

  try
  {
    MotionGenerator motion_generator(0.2, q_goal, current_state_);

    read_state_.lock();
    robot_->control(motion_generator);
    read_state_.unlock();
  }
  catch (const franka::Exception &e)
  {
    RCLCPP_WARN_STREAM(get_logger(), "Franka Exception: " << e.what());
    RCLCPP_ERROR(get_logger(), "Unable to move to target frame");

    throw CommanderError("Unable to move to target frame");
  }
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

bool RobotCommander::attempt_install(double force, double max_travel)
{
  std::unique_ptr<ForceMotionGenerator> force_motion_generator;
  franka::Model model = robot_->loadModel();
  try
  {
    force_motion_generator = std::make_unique<ForceMotionGenerator>(force, max_travel, model, current_state_);
  }
  catch (InvalidParameters &ip)
  {
    throw CommanderError(ip.what());
    return false;
  }

  try
  {
    read_state_.lock();
    robot_->control(*force_motion_generator);
    read_state_.unlock();
  }
  catch (const franka::Exception &e)
  {
    std::string ex = e.what();
    throw CommanderError("Franka Exception: " + ex);
    return false;
  }
  return force_motion_generator->get_result();
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
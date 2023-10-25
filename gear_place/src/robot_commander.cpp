#include <gear_place/robot_commander.hpp>

RobotCommander::RobotCommander(const std::string &robot_ip)
    : Node("robot_commander")
{
  // Connect to robot
  try
  {
    robot_ = std::make_unique<franka::Robot>(robot_ip, franka::RealtimeConfig::kEnforce);
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

  if (!kdl_tree_.getChain("world", "fr3_hand_tcp", chain_))
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
  camera_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("camera_angle",10);

  // ROS Timers
  joint_state_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&RobotCommander::joint_state_publish_timer_cb_, this),
      publisher_cb_group_);

  ee_pose_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&RobotCommander::ee_pose_publish_timer_cb_, this),
      publisher_cb_group_);

  camera_angle_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotCommander::camera_angle_publish_timer_cb_, this),
      publisher_cb_group_);

  // ROS Services
  move_to_named_pose_srv_ = this->create_service<gear_place_interfaces::srv::MoveToNamedPose>(
      "move_to_named_pose",
      std::bind(
          &RobotCommander::move_to_named_pose_cb_, this,
          std::placeholders::_1, std::placeholders::_2));

  move_cartesian_srv_ = this->create_service<gear_place_interfaces::srv::MoveCartesian>(
      "move_cartesian",
      std::bind(&RobotCommander::move_cartesian_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  pick_up_gear_srv_ = this->create_service<gear_place_interfaces::srv::PickUpGear>(
      "pick_up_gear",
      std::bind(&RobotCommander::pick_up_gear_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  put_gear_down_srv_ = this->create_service<gear_place_interfaces::srv::PutGearDown>(
      "put_gear_down",
      std::bind(&RobotCommander::put_gear_down_cb_, this,
                std::placeholders::_1, std::placeholders::_2));
  open_gripper_srv_ = this->create_service<gear_place_interfaces::srv::OpenGripper>(
      "open_gripper",
      std::bind(&RobotCommander::open_gripper_cb_, this,
                std::placeholders::_1, std::placeholders::_2));

  pick_up_moving_gear_srv_ = this->create_service<gear_place_interfaces::srv::PickUpMovingGear>(
      "pick_up_moving_gear",
      std::bind(&RobotCommander::pick_up_moving_gear_cb_, this,
                std::placeholders::_1, std::placeholders::_2));
  
  put_down_force_srv_ = this->create_service<gear_place_interfaces::srv::PutDownForce>(
      "put_down_force",
      std::bind(&RobotCommander::put_down_force_cb_, this,
                std::placeholders::_1, std::placeholders::_2));
  
  get_camera_angle_srv_ = this->create_service<gear_place_interfaces::srv::GetCameraAngle>(
      "get_camera_angle",
      std::bind(&RobotCommander::get_camera_angle_cb_, this,
                std::placeholders::_1, std::placeholders::_2));
  
  move_cartesian_angle_srv_ = this->create_service<gear_place_interfaces::srv::MoveCartesianAngle>(
      "move_cartesian_angle",
      std::bind(&RobotCommander::move_cartesian_angle_cb_, this,
                std::placeholders::_1, std::placeholders::_2));
  
  rotate_single_joint_srv_ = this->create_service<gear_place_interfaces::srv::RotateSingleJoint>(
    "rotate_single_joint",
    std::bind(&RobotCommander::rotate_single_joint_cb_, this,
              std::placeholders::_1, std::placeholders::_2));
  
  move_to_joint_position_srv_ = this->create_service<gear_place_interfaces::srv::MoveToJointPosition>(
    "move_to_joint_position",
    std::bind(&RobotCommander::move_to_joint_position_cb_, this,
              std::placeholders::_1, std::placeholders::_2));
  
  get_joint_positions_srv_ = this->create_service<gear_place_interfaces::srv::GetJointPositions>(
    "get_joint_positions",
    std::bind(&RobotCommander::get_joint_positions_cb_, this,
              std::placeholders::_1, std::placeholders::_2));
  
  move_cartesian_smooth_srv_ = this->create_service<gear_place_interfaces::srv::MoveCartesianSmooth>(
    "move_cartesian_smooth",
    std::bind(&RobotCommander::move_cartesian_smooth_cb_, this,
              std::placeholders::_1, std::placeholders::_2));

  std::srand(std::time(0)); // use current time as seed for random generator
}

void RobotCommander::move_to_named_pose_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::MoveToNamedPose::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::MoveToNamedPose::Response> response)
{
  /*
  Accepts the name of a pose made in robot_commander.hpp and moves the robot to that pose
  */
  if (named_joint_positions_.find(request->pose) != named_joint_positions_.end())
  {
    try
    {
      MotionGenerator motion_generator(0.3, named_joint_positions_[request->pose], current_state_);

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
  /*
  Publishes the current state of each joint
  */
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
  /*
  Publishes the current state of the end effector
  */
  ee_pose_.header.frame_id = "world";
  ee_pose_.header.stamp = this->get_clock()->now();

  ee_pose_.pose = solve_fk();

  ee_pose_pub_->publish(ee_pose_);
}

void RobotCommander::camera_angle_publish_timer_cb_()
{
  /*
  Publishes the current angle of the camera in relation to the base
  */
  camera_angle_.data = get_camera_angle();
  camera_angle_pub_->publish(camera_angle_);
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
    const std::shared_ptr<gear_place_interfaces::srv::MoveCartesian::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::MoveCartesian::Response> response)
{
  /*
  Callback for the move_robot_cartesian function
  */
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
  /*
  Makes an CartesianMotionGenerator object with the paramaters and starts a control loop with it
  */
  robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
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
    throw CommanderError("Franka Exception in cartesian motion: " + ex);
  }
}

void RobotCommander::move_cartesian_angle_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::MoveCartesianAngle::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::MoveCartesianAngle::Response> response)
{
  /*
  Callback for the move_robot_cartesian_angle function
  */
  try
  {
    move_robot_cartesian_angle(request->x, request->y, request->z, request->max_velocity, request->acceleration,request->angle);
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

void RobotCommander::move_robot_cartesian_angle(double x, double y, double z, double maximum_velocity, double acceleration, double angle)
{
  /*
  Makes an CartesianMotionGenerator object with the paramaters and starts a control loop with it
  */
  robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  std::unique_ptr<CartesianMotionGenerator> cartesian_motion_generator;
  double x_val = x * cos(angle) - y * sin(angle);
  double y_val = x * sin(angle) + y * cos(angle);
  try
  {
    cartesian_motion_generator = std::make_unique<CartesianMotionGenerator>(x_val, y_val, z, maximum_velocity, acceleration, current_state_);
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
    throw CommanderError("Franka Exception in cartesian motion: " + ex);
  }
}

void RobotCommander::put_down_force_cb_(const std::shared_ptr<gear_place_interfaces::srv::PutDownForce::Request> request,
                        std::shared_ptr<gear_place_interfaces::srv::PutDownForce::Response> response)
{
  /*
  Uses the force generator to put down the gear until it makes contact with the surface
  */
  double angle = get_camera_angle();
  double time = 1.0;
  try{
    sleep(1.0);
    put_down_force(request->force);
    sleep(1.0);
    while(time>=0.15){
      move_robot_cartesian_angle(0.0013,0.0,-0.01,default_velocity_,0.5, angle);
      sleep(1);
      auto start = std::chrono::high_resolution_clock::now();
      put_down_force(request->force);
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> duration = end - start;
      time = duration.count();
    }
    open_gripper();
    move_robot_cartesian_smooth(0.0,0.0,0.01, default_velocity_, default_acceleration_);
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

void RobotCommander::put_down_force(double force)
{
  /*
  Uses the force generator to put down the gear until it makes contact with the surface
  */
  robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  std::unique_ptr<ForceMotionGenerator> force_motion_generator;
  franka::Model model = robot_->loadModel();
  
  try
  {
    read_state_.lock();
    current_state_ = robot_->readOnce();
    read_state_.unlock();
    force_motion_generator = std::make_unique<ForceMotionGenerator>(force, model, current_state_);
  }
  catch(InvalidParameters &ip)
  {
    RCLCPP_ERROR(get_logger(),"Could not make force motion generator");
    throw CommanderError(ip.what());
  }
  
  try
  {
    read_state_.lock();
    robot_->control(*force_motion_generator);
    read_state_.unlock();
  }
  catch (const franka::Exception &e)
  {
    read_state_.unlock();
    RCLCPP_ERROR(get_logger(), e.what());
  }
}

void RobotCommander::pick_up_gear_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::PickUpGear::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::PickUpGear::Response> response)
{
  /*
  Moves to above the object, opens the gripper, moves down to the object, grasps it, and picks it up.
  Returns false if the object is not grasped or if there is an issue moving to it.
  */
  try
  {
    if (request->x != 0 && request->y != 0){
      move_robot_cartesian_smooth(request->x, request->y, 0, default_velocity_, default_acceleration_);
      sleep(wait_time_);
    }
    move_robot_cartesian_smooth(0, 0, request->z, default_velocity_, default_acceleration_);
    grasp_object(request->object_width);
    if(request->default_up){
      move_robot_cartesian_smooth(0, 0, 0.247, default_velocity_, default_acceleration_);
    }
    else{
      move_robot_cartesian_smooth(0, 0, -1 * request->z + 0.0022, default_velocity_, default_acceleration_);
    }
  }
  catch (CommanderError &e)
  {
    std::string err = e.what();
    RCLCPP_ERROR(get_logger(), err.c_str());
    response->success = false;
    return;
  }

  gripper_state_ = gripper_->readOnce();
  if (!gripper_state_.is_grasped)
  {
    RCLCPP_WARN(get_logger(), "Object was not grasped");
    response->success = false;
  }
  response->success = true;
}

void RobotCommander::put_gear_down_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::PutGearDown::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::PutGearDown::Response> response)
{
  /*
  Moves down using the given height with an offset up, opens the gripper, and moves back up.
  */
  try
  {
    move_robot_cartesian_smooth(0, 0, request->z + 0.002, default_velocity_, default_acceleration_);
    sleep(wait_time_);
    open_gripper();
    sleep(wait_time_);
    move_robot_cartesian_smooth(0, 0, -1 * request->z - 0.002, default_velocity_, default_acceleration_);
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


void RobotCommander::pick_up_moving_gear_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::PickUpMovingGear::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::PickUpMovingGear::Response> response)
{
  /*
  Moves to above the object, opens the gripper, moves down to the object, grasps it, and picks it up.
  Returns false if the object is not grasped or if there is an issue moving to it.
  */
  try
  {
    move_robot_cartesian_smooth(0, 0, request->z*3/4, default_velocity_, default_acceleration_);
    move_robot_cartesian_angle(request->x, request->y, 0, default_velocity_, default_acceleration_, request->angle);
    move_robot_cartesian_smooth(0, 0, request->z/4+0.001, default_velocity_, default_acceleration_);
    grasp_object(request->object_width);
    move_robot_cartesian_smooth(0, 0, -1 * request->z-0.001, default_velocity_, default_acceleration_);
  }
  catch (CommanderError &e)
  {
    std::string err = e.what();
    RCLCPP_ERROR(get_logger(), err.c_str());
    response->success = false;
    return;
  }

  gripper_state_ = gripper_->readOnce();
  if (!gripper_state_.is_grasped)
  {
    RCLCPP_WARN(get_logger(), "Object was not grasped");
    response->success = false;
  }
  response->success = true;
}

void RobotCommander::open_gripper_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::OpenGripper::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::OpenGripper::Response> response)
{
  /*
  Opens the gripper
  */
  (void) request;
  try
  {
    open_gripper();
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

void RobotCommander::open_gripper()
{
  /*
  Opens the gripper of the robot to almost the full amount
  */
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
  /*
  Closes the gripper to the amount where it will pick up the object
  */
  if (gripper_state_.max_width < object_width)
  {
    throw CommanderError("Object width is larger than gripper max width");
  }
  if (!gripper_->grasp(object_width, gripper_speed_, gripper_force_))
  {
    for(int i = 0; i < 2; i++){
      open_gripper();
      RCLCPP_ERROR(get_logger(),"Unable to grasp object");
      move_robot_cartesian_smooth(0.009,0.0,-0.001, default_velocity_, default_acceleration_);
      if (!gripper_->grasp(object_width, gripper_speed_, gripper_force_))
      {
          open_gripper();
          RCLCPP_ERROR(get_logger(),"Unable to grasp object");
      }
      else{
        return;
      }
    }
  }
  else{
    return;
  }
  throw(std::out_of_range("Unable to grasp object"));
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

double RobotCommander::get_camera_angle(){
  return joint_state_msg_.position[6]-0.816665-joint_state_msg_.position[0];
}

void RobotCommander::get_camera_angle_cb_(const std::shared_ptr<gear_place_interfaces::srv::GetCameraAngle::Request>request,
                          std::shared_ptr<gear_place_interfaces::srv::GetCameraAngle::Response> response){
  /*
  Responds with the camera angle
  */
  (void) request;
  response->angle = camera_angle_.data;
}

void RobotCommander::rotate_single_joint_cb_(const std::shared_ptr<gear_place_interfaces::srv::RotateSingleJoint::Request> request,
                          std::shared_ptr<gear_place_interfaces::srv::RotateSingleJoint::Response> response){
    double initial_pose[7];
    for(int i = 0; i < 7; i++){
        initial_pose[i]=joint_state_msg_.position[i];
    }
    if(request->radians){
      initial_pose[request->joint]+=request->angle;
    }
    else{
      initial_pose[request->joint]+=request->angle*M_PI/180.0;
    }
    if(initial_pose[request->joint]>M_PI){
      initial_pose[request->joint] = (initial_pose[request->joint]-2*M_PI);
    }
    else if(initial_pose[request->joint]<-1*M_PI){
      initial_pose[request->joint] = (initial_pose[request->joint]+2*M_PI);
    }
    try
    {
      MotionGenerator motion_generator(0.2,
      {{initial_pose[0],initial_pose[1],initial_pose[2],
      initial_pose[3],initial_pose[4],initial_pose[5], initial_pose[6]}}
      , current_state_);

      read_state_.lock();
      robot_->control(motion_generator);
      read_state_.unlock();
    }
    catch (const franka::Exception &e)
    {
      response->success = false;
      RCLCPP_WARN_STREAM(get_logger(), "Franka Exception: " << e.what());
      response->success = false;
      return;
    }
    response->success = true;

}

void RobotCommander::move_to_joint_position_cb_(const std::shared_ptr<gear_place_interfaces::srv::MoveToJointPosition::Request> request,
                          std::shared_ptr<gear_place_interfaces::srv::MoveToJointPosition::Response> response){
    double target_pose[7];
    for(int i = 0; i < 7; i++){
        target_pose[i]=request->joint_positions[i];
    }
    try
    {
      MotionGenerator motion_generator(0.2,
      {{target_pose[0],target_pose[1],target_pose[2],
      target_pose[3],target_pose[4],target_pose[5], target_pose[6]}}
      , current_state_);

      read_state_.lock();
      robot_->control(motion_generator);
      read_state_.unlock();
    }
    catch (const franka::Exception &e)
    {
      response->success = false;
      RCLCPP_WARN_STREAM(get_logger(), "Franka Exception: " << e.what());
      response->success = false;
      return;
    }
    response->success = true;

}

void RobotCommander::get_joint_positions_cb_(const std::shared_ptr<gear_place_interfaces::srv::GetJointPositions::Request>request,
                          std::shared_ptr<gear_place_interfaces::srv::GetJointPositions::Response> response){
  /*
  Responds with the joint positions
  */
  (void) request;
  for(int i = 0; i < 7; i++){
      response->joint_positions[i]=joint_state_msg_.position[i];
  }
}

void RobotCommander::move_cartesian_smooth_cb_(
    const std::shared_ptr<gear_place_interfaces::srv::MoveCartesianSmooth::Request> request,
    std::shared_ptr<gear_place_interfaces::srv::MoveCartesianSmooth::Response> response)
{
  /*
  Callback for the move_robot_cartesian_smooth function
  */
  try
  {
    move_robot_cartesian_smooth(request->x, request->y, request->z, request->max_velocity, request->acceleration);
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

void RobotCommander::move_robot_cartesian_smooth(double x, double y, double z, double maximum_velocity, double acceleration)
{
  /*
  Makes an SmoothCartesianMotionGenerator object with the paramaters and starts a control loop with it
  */
  robot_->setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});
  std::unique_ptr<SmoothCartesianMotionGenerator> smooth_cartesian_motion_generator;
  try
  {
    smooth_cartesian_motion_generator = std::make_unique<SmoothCartesianMotionGenerator>(x, y, z, maximum_velocity, acceleration, current_state_);
  }
  catch (InvalidParameters &ip)
  {
    throw CommanderError(ip.what());
  }

  try
  {
    read_state_.lock();
    robot_->control(*smooth_cartesian_motion_generator);
    read_state_.unlock();
  }
  catch (const franka::Exception &e)
  {
    std::string ex = e.what();
    throw CommanderError("Franka Exception in smooth cartesian motion: " + ex);
  }
}
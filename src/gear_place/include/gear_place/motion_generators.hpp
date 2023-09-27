#pragma once

#include <rclcpp/rclcpp.hpp>

#include <array>

#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <kdl/frames.hpp>
#include <franka/model.h>

#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <cmath>

#include <cmath>
#include <Eigen/Core>
#include <mutex>

class InvalidParameters : public std::exception
{
public:
  std::string what()
  {
    return "The velocity profile is not possible for the given parameters";
  }
};

class MotionGenerator
{
public:
  MotionGenerator(double speed_factor, const std::array<double, 7> q_goal, franka::RobotState &state);
  franka::JointPositions operator()(const franka::RobotState &robot_state, franka::Duration period);

private:
  using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
  using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector7d *delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector7d q_goal_;

  Vector7d q_start_;
  Vector7d delta_q_;

  Vector7d dq_max_sync_;
  Vector7d t_1_sync_;
  Vector7d t_2_sync_;
  Vector7d t_f_sync_;
  Vector7d q_1_;

  double time_ = 0.0;

  Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();
  Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();
  Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();

  franka::RobotState &state_;
};

class CartesianMotionGenerator
{
public:
  CartesianMotionGenerator(double x, double y, double z, double maximum_velocity, double acceleration, franka::RobotState &state);
  franka::CartesianPose operator()(const franka::RobotState &robot_state, franka::Duration period);

private:
  double time_ = 0.0;
  double v_max_;
  double a_;
  geometry_msgs::msg::Vector3 d_;
  double t1_, t2_, t3_;
  double d_norm_;
  std::array<double, 16> initial_pose_;

  geometry_msgs::msg::Vector3 calculate_displacement(double time);
  double norm(geometry_msgs::msg::Vector3);
  bool is_finished() { return time_ >= t3_; };

  franka::RobotState &state_;
};

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal, franka::RobotState &state)
    : q_goal_(q_goal.data()), state_(state)
{
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector7d *delta_q_d) const
{
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  for (size_t i = 0; i < 7; i++)
  {
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished)
    {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    }
    else
    {
      if (t < t_1_sync_[i])
      {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      }
      else if (t >= t_1_sync_[i] && t < t_2_sync_[i])
      {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      }
      else if (t >= t_2_sync_[i] && t < t_f_sync_[i])
      {
        (*delta_q_d)[i] =
                          delta_q_[i] + 0.5 *(1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                          (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                          std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                          (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                          dq_max_sync_[i] * sign_delta_q[i];
      }
      else
      {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x)
                     { return x; });
}

void MotionGenerator::calculateSynchronizedValues()
{
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 7; i++)
  {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished)
    {
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i])))
      {
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 7; i++)
  {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished)
    {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0)
      {
        delta = 0.0;
      }
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::JointPositions MotionGenerator::operator()(const franka::RobotState &robot_state,
                                                   franka::Duration period)
{
  time_ += period.toSec();

  if (time_ == 0.0)
  {
    q_start_ = Vector7d(robot_state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  state_ = robot_state;

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}

CartesianMotionGenerator::CartesianMotionGenerator(double x, double y, double z, double maximum_velocity, double acceleration, franka::RobotState &state)
    : v_max_(maximum_velocity), a_(acceleration), state_(state)
{
  d_.x = x;
  d_.y = y;
  d_.z = z;
  d_norm_ = norm(d_);
  t1_ = v_max_ / a_;
  t2_ = d_norm_ / v_max_;
  t3_ = t1_ + t2_;
  if (t1_ * v_max_ > d_norm_)
  {
    if(abs(x)+abs(y)+abs(z)!=0.1){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entered values not possible. Lowering v_max_");
    }
    v_max_ = d_norm_ / t1_ * 0.9;
    t1_ = v_max_ / a_;
    t2_ = d_norm_ / v_max_;
    t3_ = t1_ + t2_;
  }
}

geometry_msgs::msg::Vector3 CartesianMotionGenerator::calculate_displacement(double time)
{
  /*
  Calculates the next coordinates for the movement
  */
  geometry_msgs::msg::Vector3 delta_vals;
  double delta;
  if (time <= t1_)
  {
    delta = a_ * pow(time, 2) * 0.5;
  }
  else if (time <= t2_)
  {
    delta = a_ * pow(t1_, 2) * 0.5 + v_max_ * (time - t1_);
  }
  else if (time < t3_)
  {
    delta = a_ * pow(t1_, 2) * 0.5 + v_max_ * (t2_ - t1_) + ((a_ * pow(t1_, 2) * 0.5) - ((v_max_ - a_ * (time - t2_)) * (t3_ - time) * 0.5));
  }
  else
  {
    return d_;
  }
  delta_vals.x = delta / d_norm_ * d_.x;
  delta_vals.y = delta / d_norm_ * d_.y;
  delta_vals.z = delta / d_norm_ * d_.z;
  return delta_vals;
}

double CartesianMotionGenerator::norm(geometry_msgs::msg::Vector3 v)
{
  /*
  For calculating the total distance
  */
  return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

franka::CartesianPose CartesianMotionGenerator::operator()(const franka::RobotState &robot_state, franka::Duration period)
{
  /*
  Function used for the control loop
  Uses the calculate_displacement function above to calculate the next distance and move to it
  */
  time_ += period.toSec();

  if (time_ == 0.0)
  {
    initial_pose_ = robot_state.O_T_EE_c;
  }

  state_ = robot_state;

  std::array<double, 16> new_pose = initial_pose_;

  geometry_msgs::msg::Vector3 delta_vals = calculate_displacement(time_);

  new_pose[12] += delta_vals.x;
  new_pose[13] += delta_vals.y;
  new_pose[14] += delta_vals.z;

  if (is_finished())
  {
    return franka::MotionFinished(new_pose);
  }

  return new_pose;
}

//----------------------------------------------------------------------
//                  Force Motion Generator
//----------------------------------------------------------------------

class ForceMotionGenerator
{
public:
  ForceMotionGenerator(double force, franka::Model &model, franka::RobotState &state);
  franka::Torques operator()(const franka::RobotState &robot_state, franka::Duration period);

private:
  double time_ = 0.0;
  double time_limit_ = 10;
  double desired_force = 5.0;
  const double k_p = 1.0;
  const double k_i = 2.0;
  const double filter_gain = 0.001;
  double force_;
  double max_travel_ = 0.005;
  int counter_ = 0;

  double desired_mass = 0.0;
  double target_mass = 1.0;

  std::array<double, 7> gravity_array;
  Eigen::Vector3d initial_position_;

  franka::Model &model_;
  franka::RobotState &state_;
  franka::RobotState initial_state_;

  Eigen::VectorXd initial_tau_ext_;
  Eigen::VectorXd tau_error_integral_;
};

ForceMotionGenerator::ForceMotionGenerator(double force, franka::Model &model, franka::RobotState &state)
    : force_(force), model_(model), state_(state), initial_tau_ext_(7), tau_error_integral_(7)
{
  gravity_array = model_.gravity(state_);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(state_.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());

  initial_tau_ext_ = initial_tau_measured - initial_gravity;
  tau_error_integral_.setZero();
}

franka::Torques ForceMotionGenerator::operator()(const franka::RobotState &robot_state,
                                                 franka::Duration period)
{
  auto current_position = Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                      robot_state.O_T_EE[14]);

  time_ += period.toSec();
  if (time_ == 0.0) {
    initial_position_ = current_position;
  }
  if (time_ > 0 && (current_position - initial_position_).norm() > 0.01) {
    throw std::runtime_error("Aborting; too far away from starting pose!");
  }
  // get state variables
  std::array<double, 42> jacobian_array =
      model_.zeroJacobian(franka::Frame::kEndEffector, robot_state);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(2) = desired_mass * -9.81;
  tau_ext << tau_measured - gravity - initial_tau_ext_;
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_integral_ += period.toSec() * (tau_d - tau_ext);
  // FF + PI control
  tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral_;
  // Smoothly update the mass to reach the desired target value
  desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;
  std::array<double, 7> tau_d_array{};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;
  counter_+=1;
  if ((counter_==100)){
    std::cout<<current_position[2]-initial_position_[2]<<std::endl;
  }
  if ((abs(current_position[2]-initial_position_[2])<=0.0001 || (current_position[2]-initial_position_[2]>0.0)) && counter_>=100){
    std::cout <<"100"<<std::endl;
    return franka::MotionFinished(franka::Torques(tau_d_array));
  }
  if (counter_>=150){
    std::cout <<"150"<<std::endl;
    return franka::MotionFinished(franka::Torques(tau_d_array));
  }
  return tau_d_array;
}
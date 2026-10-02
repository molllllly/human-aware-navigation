#include "air_navigation/python_trajectory_generator.h"

#include "air_navigation/py_interface.h"
#include "air_navigation/python_interpreter.h"
#include "nav_2d_utils/parameters.hpp"

using namespace air_navigation;

void PythonTrajectoryGenerator::initialize(const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & plugin_name)
{
  std::string module_name = nav_2d_utils::searchAndGetParam(
    nh,
    plugin_name + ".air_navigation::PythonTrajectoryGenerator.module_name", std::string());
  std::string class_name = nav_2d_utils::searchAndGetParam(
    nh,
    plugin_name + ".air_navigation::PythonTrajectoryGenerator.class_name", std::string());

  RCLCPP_INFO_STREAM(nh->get_logger(), "Python trajectory generator will use: " << plugin_name);

  RCLCPP_INFO_STREAM(nh->get_logger(), "Python trajectory generator will use: " << module_name << "." << class_name);
  m_python_trajectory_generator_interface = air_navigation::python_interpreter::instance()->create_trajectory_generator(module_name, class_name);;
  if(not m_python_trajectory_generator_interface)
  {
    RCLCPP_ERROR_STREAM(nh->get_logger(), "Failed to trajectory generator critic!");
  } else {
    python_lock pl;
    m_python_trajectory_generator_interface->initialize(plugin_name);
  }


}

void PythonTrajectoryGenerator::reset()
{
  python_lock pl;
  if(m_python_trajectory_generator_interface) m_python_trajectory_generator_interface->reset();

}

void PythonTrajectoryGenerator::startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity)
{
  python_lock pl;
  if(m_python_trajectory_generator_interface)
  {
    py_interface::Twist2D py_current_velocity;
    py_interface::copy_twist_2d(current_velocity, &py_current_velocity);
    m_python_trajectory_generator_interface->startNewIteration(py_current_velocity);
  }
}

bool PythonTrajectoryGenerator::hasMoreTwists()
{
  python_lock pl;
  if(m_python_trajectory_generator_interface) return m_python_trajectory_generator_interface->hasMoreTwists();
  return false;
}

nav_2d_msgs::msg::Twist2D PythonTrajectoryGenerator::nextTwist()
{
  python_lock pl;
  nav_2d_msgs::msg::Twist2D twist;
  if(m_python_trajectory_generator_interface)
  {
    py_interface::Twist2D py_twist = m_python_trajectory_generator_interface->nextTwist();
    py_interface::copy_twist_2d(py_twist, &twist);
  }
  return twist;
}

dwb_msgs::msg::Trajectory2D PythonTrajectoryGenerator::generateTrajectory(const geometry_msgs::msg::Pose2D & start_pose, const nav_2d_msgs::msg::Twist2D & start_vel, const nav_2d_msgs::msg::Twist2D & cmd_vel)
{
  python_lock pl;
  dwb_msgs::msg::Trajectory2D trajectory;
  if(m_python_trajectory_generator_interface)
  {
    py_interface::Pose2D py_start_pose;
    py_interface::Twist2D py_start_vel, py_cmd_vel;

    py_interface::copy_pose_2d(start_pose, &py_start_pose);
    py_interface::copy_twist_2d(start_vel, &py_start_vel);
    py_interface::copy_twist_2d(cmd_vel, &py_cmd_vel);
    
    py_interface::Trajectory2D py_trajectory = m_python_trajectory_generator_interface->generateTrajectory(py_start_pose, py_start_vel, py_cmd_vel);
    py_interface::copy_trajectory_2d<geometry_msgs::msg::Pose2D, builtin_interfaces::msg::Duration>(py_trajectory, &trajectory);
  }
  return trajectory;
}

void PythonTrajectoryGenerator::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  python_lock pl;
  if(m_python_trajectory_generator_interface) m_python_trajectory_generator_interface->setSpeedLimit(speed_limit, percentage);
}

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(air_navigation::PythonTrajectoryGenerator, dwb_core::TrajectoryGenerator)

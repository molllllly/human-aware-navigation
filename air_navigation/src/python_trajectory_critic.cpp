#include "air_navigation/python_trajectory_critic.h"

#include "air_navigation/py_interface.h"
#include "air_navigation/python_interpreter.h"
#include "nav_2d_utils/parameters.hpp"

using namespace air_navigation;

void PythonTrajectoryCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string module_name = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".module_name", std::string());
  std::string class_name = nav_2d_utils::searchAndGetParam(
    node,
    dwb_plugin_name_ + "." + name_ + ".class_name", std::string());

  RCLCPP_INFO_STREAM(node->get_logger(), "Python critic will use: " << module_name << "." << class_name);
  m_python_trajectory_generator_interface = air_navigation::python_interpreter::instance()->create_trajectory_critic(module_name, class_name);;
  if(not m_python_trajectory_generator_interface)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to construct critic!");
  } else {
    python_lock pl;
    m_python_trajectory_generator_interface->onInit();
  }

}

void PythonTrajectoryCritic::reset()
{
  python_lock pl;
  if(m_python_trajectory_generator_interface) m_python_trajectory_generator_interface->reset();
}

bool PythonTrajectoryCritic::prepare(const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel, const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan)
{
  python_lock pl;
  if(m_python_trajectory_generator_interface)
  {
    py_interface::Pose2D py_pose;
    py_interface::Twist2D py_vel;
    py_interface::Pose2D py_goal;
    py_interface::Path2D py_global_plan;

    py_interface::copy_pose_2d(pose, &py_pose);
    py_interface::copy_twist_2d(vel, &py_vel);
    py_interface::copy_pose_2d(goal, &py_goal);
    py_interface::copy_path_2d<py_interface::Pose2D>(global_plan, &py_global_plan);

    return m_python_trajectory_generator_interface->prepare(py_pose, py_vel, py_goal, py_global_plan);
  }
  return false;
}
double PythonTrajectoryCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  python_lock pl;
  if(m_python_trajectory_generator_interface)
  {
    py_interface::Trajectory2D py_traj;
    py_interface::copy_trajectory_2d<py_interface::Pose2D, py_interface::Time>(traj, &py_traj);
    return m_python_trajectory_generator_interface->scoreTrajectory(py_traj);
  }
  return 0.0;
}
void PythonTrajectoryCritic::debrief(const nav_2d_msgs::msg::Twist2D & vel)
{
  python_lock pl;
  if(m_python_trajectory_generator_interface)
  {
    py_interface::Twist2D py_vel;
    py_interface::copy_twist_2d(vel, &py_vel);
    return m_python_trajectory_generator_interface->debrief(py_vel);
  }

}

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(air_navigation::PythonTrajectoryCritic, dwb_core::TrajectoryCritic)

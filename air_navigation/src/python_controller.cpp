
#include <algorithm>
#include <string>
#include <memory>
#include <iostream>

#include "nav2_util/node_utils.hpp"
#include "air_navigation/python_controller.h"
#include "air_navigation/py_interface.h"
#include "nav2_util/geometry_utils.hpp"

#include "air_navigation/python_interpreter.h"

using namespace air_navigation;

void PythonController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  auto node = parent.lock();
  RCLCPP_ERROR_STREAM(node->get_logger(), "****************** Configuring Python controller");

  nav2_util::declare_parameter_if_not_declared(node, name + ".module", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node, name + ".class", rclcpp::ParameterValue(""));

  std::string module_name, class_name;

  node->get_parameter(name + ".module", module_name);
  node->get_parameter(name + ".class", class_name);

  RCLCPP_INFO_STREAM(node->get_logger(), "Python controller will use: " << module_name << "." << class_name);
  m_python_controller_interface = air_navigation::python_interpreter::instance()->create_controller(module_name, class_name);;
  if(not m_python_controller_interface)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to construct controller!");
  } else {
    python_lock pl;
    m_python_controller_interface->configure(name);
  }
}

void PythonController::cleanup()
{
  python_lock pl;
  if(m_python_controller_interface) m_python_controller_interface->cleanup();
}

void PythonController::activate()
{
  python_lock pl;
  if(m_python_controller_interface) m_python_controller_interface->activate();
}

void PythonController::deactivate()
{
  python_lock pl;
  if(m_python_controller_interface) m_python_controller_interface->deactivate();
}


geometry_msgs::msg::TwistStamped PythonController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & twist,
    nav2_core::GoalChecker * goal)
{
  python_lock pl;
  
  if(m_python_controller_interface)
  {
    py_interface::PoseStamped py_pose;
    py_interface::copy_pose_stamped(pose, &py_pose);
    py_interface::Twist py_twis;
    py_interface::copy_twist(twist, &py_twis);
    py_interface::GoalChecker gc(goal);
    py_interface::TwistStamped py_twis_stamped = m_python_controller_interface->computeVelocityCommands(py_pose, py_twis, &gc);
    geometry_msgs::msg::TwistStamped twist_stamped;
    py_interface::copy_header(py_twis_stamped.header, &twist_stamped.header);
    py_interface::copy_twist(py_twis_stamped.twist, &twist_stamped.twist);

    return twist_stamped;
  } else {
    return geometry_msgs::msg::TwistStamped();
  }
}

void PythonController::setPlan(const nav_msgs::msg::Path & path)
{
  python_lock pl;

  if(m_python_controller_interface)
  {
    py_interface::Path py_path;
    py_interface::copy_path<py_interface::PoseStamped>(path, &py_path);
    m_python_controller_interface->setPlan(py_path);
  }
}

void PythonController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  python_lock pl;
  if(m_python_controller_interface)
  {
    m_python_controller_interface->setSpeedLimit(speed_limit, percentage);
  }
}

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(air_navigation::PythonController, nav2_core::Controller)

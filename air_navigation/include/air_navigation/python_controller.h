#pragma once

#include <string>
#include <vector>
#include <memory>

#include "nav2_core/controller.hpp"


namespace air_navigation
{

namespace py_interface
{
class Controller;
}

class PythonController : public nav2_core::Controller
{
public:
  PythonController() = default;
  ~PythonController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);


  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  py_interface::Controller* m_python_controller_interface = nullptr;
};

}  // namespace nav2_pure_pursuit_controller

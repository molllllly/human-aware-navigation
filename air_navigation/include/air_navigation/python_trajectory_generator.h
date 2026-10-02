#include <dwb_core/trajectory_generator.hpp>

namespace air_navigation
{
  namespace py_interface
  {
    class TrajectoryGenerator;
  }

  class PythonTrajectoryGenerator : public dwb_core::TrajectoryGenerator
  {
  public:
    PythonTrajectoryGenerator() = default;
    ~PythonTrajectoryGenerator() = default;
    void initialize(const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & plugin_name) override;
    void reset() override;
    void startNewIteration(const nav_2d_msgs::msg::Twist2D & current_velocity) override;
    bool hasMoreTwists() override;
    nav_2d_msgs::msg::Twist2D nextTwist() override;
    dwb_msgs::msg::Trajectory2D generateTrajectory(const geometry_msgs::msg::Pose2D & start_pose, const nav_2d_msgs::msg::Twist2D & start_vel, const nav_2d_msgs::msg::Twist2D & cmd_vel) override;
    void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  private:
    py_interface::TrajectoryGenerator* m_python_trajectory_generator_interface = nullptr;
  };
}

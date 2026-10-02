#include <dwb_core/trajectory_critic.hpp>

namespace air_navigation
{
  namespace py_interface
  {
    class TrajectoryCritic;
  }

  class PythonTrajectoryCritic : public dwb_core::TrajectoryCritic
  {
  public:
    PythonTrajectoryCritic() = default;
    ~PythonTrajectoryCritic() = default;
    void onInit() override;
    void reset() override;
    bool prepare(const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & vel, const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & global_plan) override;
    double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;
    void debrief(const nav_2d_msgs::msg::Twist2D &) override;
  private:
    py_interface::TrajectoryCritic* m_python_trajectory_generator_interface = nullptr;
  };
}

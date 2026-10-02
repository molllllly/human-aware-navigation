#include <iostream>
#include <string>
#include <vector>
#include <nav2_core/goal_checker.hpp>

namespace air_navigation::py_interface
{
  struct Time
  {
    int32_t sec;
    uint32_t nanosec;
  };
  struct Header
  {
    Time stamp;
    std::string frame_id;
  };
  struct Vector3 { double x; double y; double z; };
  struct Vector4 : public Vector3 { double w; };
  struct Pose {
    Vector3 position;
    Vector4 orientation;
  };
  struct PoseStamped
  {
    Header header;
    Pose pose;
  };
  struct Twist { Vector3 linear; Vector3 angular; };
  struct TwistStamped { Header header; Twist twist; };
  struct Path { Header header; std::vector<PoseStamped> poses; };
  struct Pose2D { double x, y, theta; };
  struct Twist2D { double x, y, theta; };
  struct Path2D { Header header;  std::vector<Pose2D> poses; };
  struct Trajectory2D { Twist2D velocity; std::vector<Time> time_offsets; std::vector<Pose2D> poses; };

  template<typename _TS_, typename _TD_>
  inline void copy_header(const _TS_& s, _TD_* d)
  {
    d->stamp.sec = s.stamp.sec;
    d->stamp.nanosec = s.stamp.nanosec;
    d->frame_id = s.frame_id;
  }
  template<typename _TS_, typename _TD_>
  inline void copy_vector3(const _TS_& s, _TD_* d)
  {
    d->x = s.x;
    d->y = s.y;
    d->z = s.z;
  }
  template<typename _TS_, typename _TD_>
  inline void copy_twist(const _TS_& s, _TD_* d)
  {
    copy_vector3(s.linear, &d->linear);
    copy_vector3(s.angular, &d->angular);
  }
  template<typename _TS_, typename _TD_>
  inline void copy_pose(const _TS_& s, _TD_* d)
  {
    d->position.x = s.position.x;
    d->position.y = s.position.y;
    d->position.z = s.position.z;
    d->orientation.x = s.orientation.x;
    d->orientation.y = s.orientation.y;
    d->orientation.z = s.orientation.z;
    d->orientation.w = s.orientation.w;
  }
  template<typename _TS_, typename _TD_>
  inline void copy_pose_stamped(const _TS_& s, _TD_* d)
  {
    copy_header(s.header, &d->header);
    copy_pose(s.pose, &d->pose);
  }

  template<typename _TPS_, typename _TS_, typename _TD_>
  inline void copy_path(const _TS_& s, _TD_* d)
  {
    copy_header(s.header, &d->header);
    for(auto p : s.poses)
    {
      _TPS_ dp;
      copy_pose_stamped(p, &dp);
      d->poses.push_back(dp);
    }
  }

  template<typename _TS_, typename _TD_>
  inline void copy_pose_2d(const _TS_& s, _TD_* d)
  {
    d->x = s.x;
    d->y = s.y;
    d->theta = s.theta;
  }
  template<typename _TS_, typename _TD_>
  inline void copy_twist_2d(const _TS_& s, _TD_* d)
  {
    copy_pose_2d(s, d);
  }
  template<typename _TPS_, typename _TS_, typename _TD_>
  inline void copy_path_2d(const _TS_& s, _TD_* d)
  {
    copy_header(s.header, &d->header);
    for(auto p : s.poses)
    {
      _TPS_ dp;
      copy_pose_2d(p, &dp);
      d->poses.push_back(dp);
    }
  }  template<typename _TPS_, typename _TPT_, typename _TS_, typename _TD_>
  inline void copy_trajectory_2d(const _TS_& s, _TD_* d)
  {
    copy_twist_2d(s.velocity, &d->velocity);
    for(auto t : s.time_offsets)
    {
      _TPT_ dt;
      dt.sec = t.sec;
      dt.nanosec = t.nanosec;
      d->time_offsets.push_back(dt);
    }
    for(auto p : s.poses)
    {
      _TPS_ dp;
      copy_pose_2d(p, &dp);
      d->poses.push_back(dp);
    }
  }
  class GoalChecker
  {
  public:
    GoalChecker(nav2_core::GoalChecker* _goalChecker) : m_goalChecker(_goalChecker)
    {}
    ~GoalChecker()
    {}
    void reset()
    {
      m_goalChecker->reset();
    }

    bool isGoalReached(
      const Pose & query_pose, const Pose & goal_pose,
      const Twist & velocity)
    {
      geometry_msgs::msg::Pose ros_query_pose;
      geometry_msgs::msg::Pose ros_goal_pose;
      geometry_msgs::msg::Twist ros_velocity;

      copy_pose(query_pose, &ros_query_pose);
      copy_pose(goal_pose, &ros_goal_pose);
      copy_twist(velocity, &ros_velocity);

      return m_goalChecker->isGoalReached(ros_query_pose, ros_goal_pose, ros_velocity);
    }

    bool getTolerances(
      Pose* pose_tolerance,
      Twist* vel_tolerance)
    {
      geometry_msgs::msg::Pose ros_pose_tolerance;
      geometry_msgs::msg::Twist ros_vel_tolerance;

      bool v = m_goalChecker->getTolerances(ros_pose_tolerance, ros_vel_tolerance);

      copy_pose(ros_pose_tolerance, pose_tolerance);
      copy_twist(ros_vel_tolerance, vel_tolerance);
      return v;
    }

  private:
    nav2_core::GoalChecker* m_goalChecker;
  };

  class Controller
  {
  public:
    Controller() = default;
    virtual ~Controller() = default;

    virtual void configure(const std::string& name) = 0;

    virtual void cleanup() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;

    virtual TwistStamped computeVelocityCommands(const PoseStamped& pose, Twist& velocity, GoalChecker* goal) = 0;

    virtual void setPlan(const Path& path) = 0;

    virtual void setSpeedLimit(double speed_limit, bool percentage) = 0;

    void print(const std::string& _text)
    {
      std::cout << _text << std::endl;
    }
  };


  class TrajectoryCritic
  {
  public:
    TrajectoryCritic() = default;
    virtual ~TrajectoryCritic() = default;

    virtual void onInit() = 0;    
    virtual void reset() = 0;    
    virtual bool prepare(const Pose2D&, const Twist2D&, const Pose2D&, const Path2D&) = 0;
    virtual double scoreTrajectory(const Trajectory2D & traj) = 0;
    virtual void debrief(const Twist2D &) = 0;
  };

  class TrajectoryGenerator
  {
  public:
    TrajectoryGenerator() = default;
    virtual ~TrajectoryGenerator() = default;
    virtual void initialize(const std::string& name) = 0;
    virtual void reset() = 0;    
    virtual void startNewIteration(const Twist2D & current_velocity) = 0;
    virtual bool hasMoreTwists() = 0;
    virtual Twist2D nextTwist() = 0;
    virtual Trajectory2D generateTrajectory(const Pose2D & start_pose, const Twist2D & start_vel, const Twist2D & cmd_vel) = 0;
    virtual void setSpeedLimit(double speed_limit, bool percentage) = 0;
  };
}

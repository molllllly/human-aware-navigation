%ModuleHeaderCode
#include <air_navigation/py_interface.h>
%End

namespace air_navigation
{
  namespace py_interface
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
    struct Vector4 : public air_navigation::py_interface::Vector3 { double w; };
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
    struct Path { Header header; std::vector<air_navigation::py_interface::PoseStamped> poses; };
    struct Pose2D { double x; double y; double theta; };
    struct Twist2D { double x; double y; double theta; };
    struct Path2D { Header header;  std::vector<air_navigation::py_interface::Pose2D> poses; };
    struct Trajectory2D { Twist2D velocity; std::vector<air_navigation::py_interface::Time> time_offsets; std::vector<air_navigation::py_interface::Pose2D> poses; };

    class GoalChecker /NoDefaultCtors/
    {
    public:
      void reset();

      bool isGoalReached(
        const Pose & query_pose, const Pose & goal_pose,
        const Twist & velocity);

      bool getTolerances(
        Pose* pose_tolerance,
        Twist* vel_tolerance);
    };

    class Controller
    {
    public:
      Controller();
      virtual ~Controller();

      virtual void configure(const std::string& name) = 0;

      virtual void cleanup() = 0;
      virtual void activate() = 0;
      virtual void deactivate() = 0;

      virtual TwistStamped computeVelocityCommands(const PoseStamped& pose, Twist& velocity, GoalChecker* goal) = 0;

      virtual void setPlan(const Path& path) = 0;

      virtual void setSpeedLimit(double speed_limit, bool percentage) = 0;

      void print(const std::string& _text);
    };
    class TrajectoryCritic
    {
    public:
      TrajectoryCritic();
      virtual ~TrajectoryCritic();

      virtual void onInit() = 0;    
      virtual void reset() = 0;    
      virtual bool prepare(const Pose2D&, const Twist2D&, const Pose2D&, const Path2D&) = 0;
      virtual double scoreTrajectory(const Trajectory2D & traj) = 0;
      virtual void debrief(const Twist2D &) = 0;
    };

    class TrajectoryGenerator
    {
    public:
      TrajectoryGenerator();
      virtual ~TrajectoryGenerator();
      virtual void initialize(const std::string& name) = 0;
      virtual void reset() = 0;    
      virtual void startNewIteration(const Twist2D & current_velocity) = 0;
      virtual bool hasMoreTwists() = 0;
      virtual Twist2D nextTwist() = 0;
      virtual Trajectory2D generateTrajectory(const Pose2D & start_pose, const Twist2D & start_vel, const Twist2D & cmd_vel) = 0;
      virtual void setSpeedLimit(double speed_limit, bool percentage) = 0;
    };
  };
};

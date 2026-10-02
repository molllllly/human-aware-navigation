#include <string>

namespace air_navigation
{
  namespace py_interface
  {
    class Controller;
    class TrajectoryCritic;
    class TrajectoryGenerator;
  }
  class python_interpreter
  {
  public:
    static python_interpreter* instance();
  private:
    python_interpreter();
    ~python_interpreter();
  public:
    py_interface::Controller* create_controller(const std::string& _module, const std::string& _class);
    py_interface::TrajectoryCritic* create_trajectory_critic(const std::string& _module, const std::string& _class);
    py_interface::TrajectoryGenerator* create_trajectory_generator(const std::string& _module, const std::string& _class);
    void release_controller(py_interface::Controller* _controller);
    void acquire_lock();
    void release_lock();
  private:
    struct Private;
    Private* const d;
  };
  struct python_lock
  {
    python_lock()
    {
      acquire_lock();
    }
    ~python_lock()
    {
      release_lock();
    }

    void acquire_lock()
    {
      python_interpreter::instance()->acquire_lock();
    }
    void release_lock()
    {
      python_interpreter::instance()->release_lock();
    }
  };
}

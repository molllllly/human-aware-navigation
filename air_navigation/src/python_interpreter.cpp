#include <air_navigation/python_interpreter.h>

#include <Python.h>
#include <sip.h>

#include <rclcpp/rclcpp.hpp>
#include <dlfcn.h>

using namespace air_navigation;

namespace
{
  const sipAPIDef *get_sip_api()
  {
    static const sipAPIDef* api = (const sipAPIDef *)PyCapsule_Import("sip._C_API", 0);
    return api;
  }
}

struct python_interpreter::Private
{
  // PyThreadState* state = 0;
  PyGILState_STATE state;
  bool self_initialise = false;

  template<typename _T_>
  _T_* create_object(const std::string& _module, const std::string& _class, const char* _cpp_name);
};

python_interpreter* python_interpreter::instance()
{
  static python_interpreter pi;
  return &pi;
}

python_interpreter::python_interpreter() : d(new Private)
{
  dlopen("libpython3.8.so", RTLD_LAZY | RTLD_GLOBAL);
  if(not Py_IsInitialized())
  {
    Py_InitializeEx(0);
    if (not Py_IsInitialized()) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("python_interpreter"), "Could not initialize Python interpreter");
    }
    d->self_initialise = true;
  }
  // d->state = PyGILState_GetThisThreadState();
  PyEval_ReleaseThread(PyGILState_GetThisThreadState());
  acquire_lock(); release_lock();
}

python_interpreter::~python_interpreter()
{
  acquire_lock();
  if(Py_IsInitialized() and d->self_initialise)
  {
    Py_Finalize();
  }
  delete d;
}

template<typename _T_>
_T_* python_interpreter::Private::create_object(const std::string& _module, const std::string& _class, const char* _cpp_name)
{
  python_lock pl;
  // PyRun_SimpleString("import air_navigation");
  PyObject *pModule = PyImport_Import(PyUnicode_DecodeFSDefault(_module.c_str()));
  if(pModule)
  {
    PyObject *pFunc = PyObject_GetAttrString(pModule, _class.c_str());
    if(pFunc and PyCallable_Check(pFunc))
    {
      PyObject* pArgs = PyTuple_New(0);
      PyObject* pValue = 
      PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      Py_DECREF(pFunc);
      int isErr = 0;
      int state;
      const sipTypeDef *td = get_sip_api()->api_find_type(_cpp_name);
      _T_* controller = reinterpret_cast<_T_*>(get_sip_api()->api_convert_to_type(pValue, td, NULL, SIP_NOT_NONE, &state, &isErr));
      get_sip_api()->api_transfer_to(pValue, Py_None);
      return controller;
    } else {
      PyErr_Print();
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("python_interpreter"), "Failed to find class '" << _class << "'");
    }
    Py_DECREF(pModule);
  } else {
    PyErr_Print();
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("python_interpreter"), "Failed to load module '" << _module << "'");
  }
  return nullptr;

}

py_interface::Controller* python_interpreter::create_controller(const std::string& _module, const std::string& _class)
{
  return d->create_object<py_interface::Controller>(_module, _class, "air_navigation::py_interface::Controller");
}

py_interface::TrajectoryCritic* python_interpreter::create_trajectory_critic(const std::string& _module, const std::string& _class)
{
  return d->create_object<py_interface::TrajectoryCritic>(_module, _class, "air_navigation::py_interface::TrajectoryCritic");

}

py_interface::TrajectoryGenerator* python_interpreter::create_trajectory_generator(const std::string& _module, const std::string& _class)
{
  return d->create_object<py_interface::TrajectoryGenerator>(_module, _class, "air_navigation::py_interface::TrajectoryGenerator");
}

void python_interpreter::release_controller(py_interface::Controller* )
{
  // TODO call Py_DECREF
}

void python_interpreter::acquire_lock()
{
  d->state = PyGILState_Ensure();
}

void python_interpreter::release_lock()
{
  PyGILState_Release(d->state);
}

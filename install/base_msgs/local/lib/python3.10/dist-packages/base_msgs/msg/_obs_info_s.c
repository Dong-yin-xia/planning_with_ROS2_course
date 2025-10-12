// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from base_msgs:msg/ObsInfo.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "base_msgs/msg/detail/obs_info__struct.h"
#include "base_msgs/msg/detail/obs_info__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__msg__obs_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[32];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("base_msgs.msg._obs_info.ObsInfo", full_classname_dest, 31) == 0);
  }
  base_msgs__msg__ObsInfo * ros_message = _ros_message;
  {  // obs_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "obs_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obs_length = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // obs_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "obs_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->obs_width = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l
    PyObject * field = PyObject_GetAttrString(_pymsg, "l");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // s
    PyObject * field = PyObject_GetAttrString(_pymsg, "s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // s_2path
    PyObject * field = PyObject_GetAttrString(_pymsg, "s_2path");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s_2path = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ds_dt_2path
    PyObject * field = PyObject_GetAttrString(_pymsg, "ds_dt_2path");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ds_dt_2path = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // t_in
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_in");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->t_in = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // t_out
    PyObject * field = PyObject_GetAttrString(_pymsg, "t_out");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->t_out = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * base_msgs__msg__obs_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ObsInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.msg._obs_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ObsInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__msg__ObsInfo * ros_message = (base_msgs__msg__ObsInfo *)raw_ros_message;
  {  // obs_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obs_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obs_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // obs_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->obs_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "obs_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // s_2path
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->s_2path);
    {
      int rc = PyObject_SetAttrString(_pymessage, "s_2path", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ds_dt_2path
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ds_dt_2path);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ds_dt_2path", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_in
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->t_in);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_in", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // t_out
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->t_out);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t_out", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

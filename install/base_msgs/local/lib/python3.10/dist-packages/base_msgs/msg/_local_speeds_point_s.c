// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
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
#include "base_msgs/msg/detail/local_speeds_point__struct.h"
#include "base_msgs/msg/detail/local_speeds_point__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__msg__local_speeds_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("base_msgs.msg._local_speeds_point.LocalSpeedsPoint", full_classname_dest, 50) == 0);
  }
  base_msgs__msg__LocalSpeedsPoint * ros_message = _ros_message;
  {  // t
    PyObject * field = PyObject_GetAttrString(_pymsg, "t");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->t = PyFloat_AS_DOUBLE(field);
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
  {  // dds_dt_2path
    PyObject * field = PyObject_GetAttrString(_pymsg, "dds_dt_2path");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dds_dt_2path = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // acceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "acceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->acceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dacceleration
    PyObject * field = PyObject_GetAttrString(_pymsg, "dacceleration");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dacceleration = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * base_msgs__msg__local_speeds_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LocalSpeedsPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.msg._local_speeds_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LocalSpeedsPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__msg__LocalSpeedsPoint * ros_message = (base_msgs__msg__LocalSpeedsPoint *)raw_ros_message;
  {  // t
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->t);
    {
      int rc = PyObject_SetAttrString(_pymessage, "t", field);
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
  {  // dds_dt_2path
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dds_dt_2path);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dds_dt_2path", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // acceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->acceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "acceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dacceleration
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dacceleration);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dacceleration", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

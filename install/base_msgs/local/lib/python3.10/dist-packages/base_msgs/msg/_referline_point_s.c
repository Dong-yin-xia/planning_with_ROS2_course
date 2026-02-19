// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from base_msgs:msg/ReferlinePoint.idl
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
#include "base_msgs/msg/detail/referline_point__struct.h"
#include "base_msgs/msg/detail/referline_point__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__msg__referline_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("base_msgs.msg._referline_point.ReferlinePoint", full_classname_dest, 45) == 0);
  }
  base_msgs__msg__ReferlinePoint * ros_message = _ros_message;
  {  // pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose_stamped__convert_from_py(field, &ros_message->pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // rs
    PyObject * field = PyObject_GetAttrString(_pymsg, "rs");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rs = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rtheta
    PyObject * field = PyObject_GetAttrString(_pymsg, "rtheta");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rtheta = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rkappa
    PyObject * field = PyObject_GetAttrString(_pymsg, "rkappa");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rkappa = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // rdkappa
    PyObject * field = PyObject_GetAttrString(_pymsg, "rdkappa");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->rdkappa = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * base_msgs__msg__referline_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ReferlinePoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.msg._referline_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ReferlinePoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__msg__ReferlinePoint * ros_message = (base_msgs__msg__ReferlinePoint *)raw_ros_message;
  {  // pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose_stamped__convert_to_py(&ros_message->pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rs
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rs);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rs", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rtheta
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rtheta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rtheta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rkappa
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rkappa);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rkappa", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rdkappa
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->rdkappa);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rdkappa", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

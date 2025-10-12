// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from base_msgs:msg/PNCMap.idl
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
#include "base_msgs/msg/detail/pnc_map__struct.h"
#include "base_msgs/msg/detail/pnc_map__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool visualization_msgs__msg__marker__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * visualization_msgs__msg__marker__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool visualization_msgs__msg__marker__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * visualization_msgs__msg__marker__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool visualization_msgs__msg__marker__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * visualization_msgs__msg__marker__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__msg__pnc_map__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[30];
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
    assert(strncmp("base_msgs.msg._pnc_map.PNCMap", full_classname_dest, 29) == 0);
  }
  base_msgs__msg__PNCMap * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // road_length
    PyObject * field = PyObject_GetAttrString(_pymsg, "road_length");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->road_length = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // road_half_width
    PyObject * field = PyObject_GetAttrString(_pymsg, "road_half_width");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->road_half_width = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // left_boundary
    PyObject * field = PyObject_GetAttrString(_pymsg, "left_boundary");
    if (!field) {
      return false;
    }
    if (!visualization_msgs__msg__marker__convert_from_py(field, &ros_message->left_boundary)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // right_boundary
    PyObject * field = PyObject_GetAttrString(_pymsg, "right_boundary");
    if (!field) {
      return false;
    }
    if (!visualization_msgs__msg__marker__convert_from_py(field, &ros_message->right_boundary)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // midline
    PyObject * field = PyObject_GetAttrString(_pymsg, "midline");
    if (!field) {
      return false;
    }
    if (!visualization_msgs__msg__marker__convert_from_py(field, &ros_message->midline)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * base_msgs__msg__pnc_map__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PNCMap */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.msg._pnc_map");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PNCMap");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__msg__PNCMap * ros_message = (base_msgs__msg__PNCMap *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // road_length
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->road_length);
    {
      int rc = PyObject_SetAttrString(_pymessage, "road_length", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // road_half_width
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->road_half_width);
    {
      int rc = PyObject_SetAttrString(_pymessage, "road_half_width", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // left_boundary
    PyObject * field = NULL;
    field = visualization_msgs__msg__marker__convert_to_py(&ros_message->left_boundary);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "left_boundary", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // right_boundary
    PyObject * field = NULL;
    field = visualization_msgs__msg__marker__convert_to_py(&ros_message->right_boundary);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "right_boundary", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // midline
    PyObject * field = NULL;
    field = visualization_msgs__msg__marker__convert_to_py(&ros_message->midline);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "midline", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

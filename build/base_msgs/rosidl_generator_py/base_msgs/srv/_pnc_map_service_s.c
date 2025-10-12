// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from base_msgs:srv/PNCMapService.idl
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
#include "base_msgs/srv/detail/pnc_map_service__struct.h"
#include "base_msgs/srv/detail/pnc_map_service__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__srv__pnc_map_service__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[53];
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
    assert(strncmp("base_msgs.srv._pnc_map_service.PNCMapService_Request", full_classname_dest, 52) == 0);
  }
  base_msgs__srv__PNCMapService_Request * ros_message = _ros_message;
  {  // map_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "map_type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->map_type = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * base_msgs__srv__pnc_map_service__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PNCMapService_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.srv._pnc_map_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PNCMapService_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__srv__PNCMapService_Request * ros_message = (base_msgs__srv__PNCMapService_Request *)raw_ros_message;
  {  // map_type
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->map_type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "map_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "base_msgs/srv/detail/pnc_map_service__struct.h"
// already included above
// #include "base_msgs/srv/detail/pnc_map_service__functions.h"

bool base_msgs__msg__pnc_map__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * base_msgs__msg__pnc_map__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__srv__pnc_map_service__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[54];
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
    assert(strncmp("base_msgs.srv._pnc_map_service.PNCMapService_Response", full_classname_dest, 53) == 0);
  }
  base_msgs__srv__PNCMapService_Response * ros_message = _ros_message;
  {  // pnc_map
    PyObject * field = PyObject_GetAttrString(_pymsg, "pnc_map");
    if (!field) {
      return false;
    }
    if (!base_msgs__msg__pnc_map__convert_from_py(field, &ros_message->pnc_map)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * base_msgs__srv__pnc_map_service__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PNCMapService_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.srv._pnc_map_service");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PNCMapService_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__srv__PNCMapService_Response * ros_message = (base_msgs__srv__PNCMapService_Response *)raw_ros_message;
  {  // pnc_map
    PyObject * field = NULL;
    field = base_msgs__msg__pnc_map__convert_to_py(&ros_message->pnc_map);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pnc_map", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

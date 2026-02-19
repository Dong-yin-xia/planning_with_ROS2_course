// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from base_msgs:msg/LocalPathPoint.idl
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
#include "base_msgs/msg/detail/local_path_point__struct.h"
#include "base_msgs/msg/detail/local_path_point__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose_stamped__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose_stamped__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool base_msgs__msg__local_path_point__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[47];
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
    assert(strncmp("base_msgs.msg._local_path_point.LocalPathPoint", full_classname_dest, 46) == 0);
  }
  base_msgs__msg__LocalPathPoint * ros_message = _ros_message;
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
  {  // s
    PyObject * field = PyObject_GetAttrString(_pymsg, "s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->s = PyFloat_AS_DOUBLE(field);
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
  {  // ds_dt
    PyObject * field = PyObject_GetAttrString(_pymsg, "ds_dt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ds_dt = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dl_ds
    PyObject * field = PyObject_GetAttrString(_pymsg, "dl_ds");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dl_ds = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dl_dt
    PyObject * field = PyObject_GetAttrString(_pymsg, "dl_dt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dl_dt = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dds_dt
    PyObject * field = PyObject_GetAttrString(_pymsg, "dds_dt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dds_dt = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ddl_ds
    PyObject * field = PyObject_GetAttrString(_pymsg, "ddl_ds");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ddl_ds = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ddl_dt
    PyObject * field = PyObject_GetAttrString(_pymsg, "ddl_dt");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ddl_dt = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // theta
    PyObject * field = PyObject_GetAttrString(_pymsg, "theta");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->theta = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kappa
    PyObject * field = PyObject_GetAttrString(_pymsg, "kappa");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kappa = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dkappa
    PyObject * field = PyObject_GetAttrString(_pymsg, "dkappa");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dkappa = PyFloat_AS_DOUBLE(field);
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
PyObject * base_msgs__msg__local_path_point__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of LocalPathPoint */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("base_msgs.msg._local_path_point");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "LocalPathPoint");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  base_msgs__msg__LocalPathPoint * ros_message = (base_msgs__msg__LocalPathPoint *)raw_ros_message;
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
  {  // ds_dt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ds_dt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ds_dt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dl_ds
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dl_ds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dl_ds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dl_dt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dl_dt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dl_dt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dds_dt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dds_dt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dds_dt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ddl_ds
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ddl_ds);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ddl_ds", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ddl_dt
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ddl_dt);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ddl_dt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // theta
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->theta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "theta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kappa
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kappa);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kappa", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dkappa
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dkappa);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dkappa", field);
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

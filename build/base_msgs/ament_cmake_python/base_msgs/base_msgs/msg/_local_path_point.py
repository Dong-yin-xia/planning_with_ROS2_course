# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/LocalPathPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LocalPathPoint(type):
    """Metaclass of message 'LocalPathPoint'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('base_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'base_msgs.msg.LocalPathPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__local_path_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__local_path_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__local_path_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__local_path_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__local_path_point

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LocalPathPoint(metaclass=Metaclass_LocalPathPoint):
    """Message class 'LocalPathPoint'."""

    __slots__ = [
        '_pose',
        '_s',
        '_l',
        '_ds_dt',
        '_dl_ds',
        '_dl_dt',
        '_dds_dt',
        '_ddl_ds',
        '_ddl_dt',
        '_theta',
        '_kappa',
        '_dkappa',
        '_rs',
        '_rtheta',
        '_rkappa',
        '_rdkappa',
    ]

    _fields_and_field_types = {
        'pose': 'geometry_msgs/PoseStamped',
        's': 'double',
        'l': 'double',
        'ds_dt': 'double',
        'dl_ds': 'double',
        'dl_dt': 'double',
        'dds_dt': 'double',
        'ddl_ds': 'double',
        'ddl_dt': 'double',
        'theta': 'double',
        'kappa': 'double',
        'dkappa': 'double',
        'rs': 'double',
        'rtheta': 'double',
        'rkappa': 'double',
        'rdkappa': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.pose = kwargs.get('pose', PoseStamped())
        self.s = kwargs.get('s', float())
        self.l = kwargs.get('l', float())
        self.ds_dt = kwargs.get('ds_dt', float())
        self.dl_ds = kwargs.get('dl_ds', float())
        self.dl_dt = kwargs.get('dl_dt', float())
        self.dds_dt = kwargs.get('dds_dt', float())
        self.ddl_ds = kwargs.get('ddl_ds', float())
        self.ddl_dt = kwargs.get('ddl_dt', float())
        self.theta = kwargs.get('theta', float())
        self.kappa = kwargs.get('kappa', float())
        self.dkappa = kwargs.get('dkappa', float())
        self.rs = kwargs.get('rs', float())
        self.rtheta = kwargs.get('rtheta', float())
        self.rkappa = kwargs.get('rkappa', float())
        self.rdkappa = kwargs.get('rdkappa', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.pose != other.pose:
            return False
        if self.s != other.s:
            return False
        if self.l != other.l:
            return False
        if self.ds_dt != other.ds_dt:
            return False
        if self.dl_ds != other.dl_ds:
            return False
        if self.dl_dt != other.dl_dt:
            return False
        if self.dds_dt != other.dds_dt:
            return False
        if self.ddl_ds != other.ddl_ds:
            return False
        if self.ddl_dt != other.ddl_dt:
            return False
        if self.theta != other.theta:
            return False
        if self.kappa != other.kappa:
            return False
        if self.dkappa != other.dkappa:
            return False
        if self.rs != other.rs:
            return False
        if self.rtheta != other.rtheta:
            return False
        if self.rkappa != other.rkappa:
            return False
        if self.rdkappa != other.rdkappa:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pose(self):
        """Message field 'pose'."""
        return self._pose

    @pose.setter
    def pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'pose' field must be a sub message of type 'PoseStamped'"
        self._pose = value

    @builtins.property
    def s(self):
        """Message field 's'."""
        return self._s

    @s.setter
    def s(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s = value

    @builtins.property
    def l(self):
        """Message field 'l'."""
        return self._l

    @l.setter
    def l(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'l' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._l = value

    @builtins.property
    def ds_dt(self):
        """Message field 'ds_dt'."""
        return self._ds_dt

    @ds_dt.setter
    def ds_dt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ds_dt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ds_dt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ds_dt = value

    @builtins.property
    def dl_ds(self):
        """Message field 'dl_ds'."""
        return self._dl_ds

    @dl_ds.setter
    def dl_ds(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dl_ds' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dl_ds' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dl_ds = value

    @builtins.property
    def dl_dt(self):
        """Message field 'dl_dt'."""
        return self._dl_dt

    @dl_dt.setter
    def dl_dt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dl_dt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dl_dt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dl_dt = value

    @builtins.property
    def dds_dt(self):
        """Message field 'dds_dt'."""
        return self._dds_dt

    @dds_dt.setter
    def dds_dt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dds_dt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dds_dt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dds_dt = value

    @builtins.property
    def ddl_ds(self):
        """Message field 'ddl_ds'."""
        return self._ddl_ds

    @ddl_ds.setter
    def ddl_ds(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ddl_ds' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ddl_ds' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ddl_ds = value

    @builtins.property
    def ddl_dt(self):
        """Message field 'ddl_dt'."""
        return self._ddl_dt

    @ddl_dt.setter
    def ddl_dt(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ddl_dt' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ddl_dt' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ddl_dt = value

    @builtins.property
    def theta(self):
        """Message field 'theta'."""
        return self._theta

    @theta.setter
    def theta(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'theta' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'theta' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._theta = value

    @builtins.property
    def kappa(self):
        """Message field 'kappa'."""
        return self._kappa

    @kappa.setter
    def kappa(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'kappa' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'kappa' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._kappa = value

    @builtins.property
    def dkappa(self):
        """Message field 'dkappa'."""
        return self._dkappa

    @dkappa.setter
    def dkappa(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dkappa' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dkappa' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dkappa = value

    @builtins.property
    def rs(self):
        """Message field 'rs'."""
        return self._rs

    @rs.setter
    def rs(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rs' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rs' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rs = value

    @builtins.property
    def rtheta(self):
        """Message field 'rtheta'."""
        return self._rtheta

    @rtheta.setter
    def rtheta(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rtheta' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rtheta' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rtheta = value

    @builtins.property
    def rkappa(self):
        """Message field 'rkappa'."""
        return self._rkappa

    @rkappa.setter
    def rkappa(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rkappa' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rkappa' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rkappa = value

    @builtins.property
    def rdkappa(self):
        """Message field 'rdkappa'."""
        return self._rdkappa

    @rdkappa.setter
    def rdkappa(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'rdkappa' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'rdkappa' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._rdkappa = value

# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/LocalSpeedsPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LocalSpeedsPoint(type):
    """Metaclass of message 'LocalSpeedsPoint'."""

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
                'base_msgs.msg.LocalSpeedsPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__local_speeds_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__local_speeds_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__local_speeds_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__local_speeds_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__local_speeds_point

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LocalSpeedsPoint(metaclass=Metaclass_LocalSpeedsPoint):
    """Message class 'LocalSpeedsPoint'."""

    __slots__ = [
        '_t',
        '_s_2path',
        '_ds_dt_2path',
        '_dds_dt_2path',
        '_speed',
        '_acceleration',
        '_dacceleration',
    ]

    _fields_and_field_types = {
        't': 'double',
        's_2path': 'double',
        'ds_dt_2path': 'double',
        'dds_dt_2path': 'double',
        'speed': 'double',
        'acceleration': 'double',
        'dacceleration': 'double',
    }

    SLOT_TYPES = (
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
        self.t = kwargs.get('t', float())
        self.s_2path = kwargs.get('s_2path', float())
        self.ds_dt_2path = kwargs.get('ds_dt_2path', float())
        self.dds_dt_2path = kwargs.get('dds_dt_2path', float())
        self.speed = kwargs.get('speed', float())
        self.acceleration = kwargs.get('acceleration', float())
        self.dacceleration = kwargs.get('dacceleration', float())

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
        if self.t != other.t:
            return False
        if self.s_2path != other.s_2path:
            return False
        if self.ds_dt_2path != other.ds_dt_2path:
            return False
        if self.dds_dt_2path != other.dds_dt_2path:
            return False
        if self.speed != other.speed:
            return False
        if self.acceleration != other.acceleration:
            return False
        if self.dacceleration != other.dacceleration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def t(self):
        """Message field 't'."""
        return self._t

    @t.setter
    def t(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 't' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._t = value

    @builtins.property
    def s_2path(self):
        """Message field 's_2path'."""
        return self._s_2path

    @s_2path.setter
    def s_2path(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 's_2path' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 's_2path' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._s_2path = value

    @builtins.property
    def ds_dt_2path(self):
        """Message field 'ds_dt_2path'."""
        return self._ds_dt_2path

    @ds_dt_2path.setter
    def ds_dt_2path(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ds_dt_2path' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'ds_dt_2path' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._ds_dt_2path = value

    @builtins.property
    def dds_dt_2path(self):
        """Message field 'dds_dt_2path'."""
        return self._dds_dt_2path

    @dds_dt_2path.setter
    def dds_dt_2path(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dds_dt_2path' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dds_dt_2path' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dds_dt_2path = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._speed = value

    @builtins.property
    def acceleration(self):
        """Message field 'acceleration'."""
        return self._acceleration

    @acceleration.setter
    def acceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'acceleration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'acceleration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._acceleration = value

    @builtins.property
    def dacceleration(self):
        """Message field 'dacceleration'."""
        return self._dacceleration

    @dacceleration.setter
    def dacceleration(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'dacceleration' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'dacceleration' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._dacceleration = value

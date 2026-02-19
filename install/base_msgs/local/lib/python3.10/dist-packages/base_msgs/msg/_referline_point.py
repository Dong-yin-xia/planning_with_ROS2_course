# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/ReferlinePoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReferlinePoint(type):
    """Metaclass of message 'ReferlinePoint'."""

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
                'base_msgs.msg.ReferlinePoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__referline_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__referline_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__referline_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__referline_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__referline_point

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


class ReferlinePoint(metaclass=Metaclass_ReferlinePoint):
    """Message class 'ReferlinePoint'."""

    __slots__ = [
        '_pose',
        '_rs',
        '_rtheta',
        '_rkappa',
        '_rdkappa',
    ]

    _fields_and_field_types = {
        'pose': 'geometry_msgs/PoseStamped',
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
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import PoseStamped
        self.pose = kwargs.get('pose', PoseStamped())
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

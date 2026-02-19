# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/LocalTrajectoryPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LocalTrajectoryPoint(type):
    """Metaclass of message 'LocalTrajectoryPoint'."""

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
                'base_msgs.msg.LocalTrajectoryPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__local_trajectory_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__local_trajectory_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__local_trajectory_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__local_trajectory_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__local_trajectory_point

            from base_msgs.msg import LocalPathPoint
            if LocalPathPoint.__class__._TYPE_SUPPORT is None:
                LocalPathPoint.__class__.__import_type_support__()

            from base_msgs.msg import LocalSpeedsPoint
            if LocalSpeedsPoint.__class__._TYPE_SUPPORT is None:
                LocalSpeedsPoint.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LocalTrajectoryPoint(metaclass=Metaclass_LocalTrajectoryPoint):
    """Message class 'LocalTrajectoryPoint'."""

    __slots__ = [
        '_path_point',
        '_speed_point',
    ]

    _fields_and_field_types = {
        'path_point': 'base_msgs/LocalPathPoint',
        'speed_point': 'base_msgs/LocalSpeedsPoint',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['base_msgs', 'msg'], 'LocalPathPoint'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['base_msgs', 'msg'], 'LocalSpeedsPoint'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from base_msgs.msg import LocalPathPoint
        self.path_point = kwargs.get('path_point', LocalPathPoint())
        from base_msgs.msg import LocalSpeedsPoint
        self.speed_point = kwargs.get('speed_point', LocalSpeedsPoint())

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
        if self.path_point != other.path_point:
            return False
        if self.speed_point != other.speed_point:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def path_point(self):
        """Message field 'path_point'."""
        return self._path_point

    @path_point.setter
    def path_point(self, value):
        if __debug__:
            from base_msgs.msg import LocalPathPoint
            assert \
                isinstance(value, LocalPathPoint), \
                "The 'path_point' field must be a sub message of type 'LocalPathPoint'"
        self._path_point = value

    @builtins.property
    def speed_point(self):
        """Message field 'speed_point'."""
        return self._speed_point

    @speed_point.setter
    def speed_point(self, value):
        if __debug__:
            from base_msgs.msg import LocalSpeedsPoint
            assert \
                isinstance(value, LocalSpeedsPoint), \
                "The 'speed_point' field must be a sub message of type 'LocalSpeedsPoint'"
        self._speed_point = value

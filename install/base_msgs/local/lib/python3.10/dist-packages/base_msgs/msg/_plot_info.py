# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/PlotInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlotInfo(type):
    """Metaclass of message 'PlotInfo'."""

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
                'base_msgs.msg.PlotInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__plot_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__plot_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__plot_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__plot_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__plot_info

            from base_msgs.msg import LocalTrajectory
            if LocalTrajectory.__class__._TYPE_SUPPORT is None:
                LocalTrajectory.__class__.__import_type_support__()

            from base_msgs.msg import ObsInfo
            if ObsInfo.__class__._TYPE_SUPPORT is None:
                ObsInfo.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlotInfo(metaclass=Metaclass_PlotInfo):
    """Message class 'PlotInfo'."""

    __slots__ = [
        '_header',
        '_trajectory_info',
        '_obs_info',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'trajectory_info': 'base_msgs/LocalTrajectory',
        'obs_info': 'sequence<base_msgs/ObsInfo>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['base_msgs', 'msg'], 'LocalTrajectory'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['base_msgs', 'msg'], 'ObsInfo')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from base_msgs.msg import LocalTrajectory
        self.trajectory_info = kwargs.get('trajectory_info', LocalTrajectory())
        self.obs_info = kwargs.get('obs_info', [])

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
        if self.header != other.header:
            return False
        if self.trajectory_info != other.trajectory_info:
            return False
        if self.obs_info != other.obs_info:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def trajectory_info(self):
        """Message field 'trajectory_info'."""
        return self._trajectory_info

    @trajectory_info.setter
    def trajectory_info(self, value):
        if __debug__:
            from base_msgs.msg import LocalTrajectory
            assert \
                isinstance(value, LocalTrajectory), \
                "The 'trajectory_info' field must be a sub message of type 'LocalTrajectory'"
        self._trajectory_info = value

    @builtins.property
    def obs_info(self):
        """Message field 'obs_info'."""
        return self._obs_info

    @obs_info.setter
    def obs_info(self, value):
        if __debug__:
            from base_msgs.msg import ObsInfo
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, ObsInfo) for v in value) and
                 True), \
                "The 'obs_info' field must be a set or sequence and each value of type 'ObsInfo'"
        self._obs_info = value

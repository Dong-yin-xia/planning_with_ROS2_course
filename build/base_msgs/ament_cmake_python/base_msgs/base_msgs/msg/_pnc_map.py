# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/PNCMap.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PNCMap(type):
    """Metaclass of message 'PNCMap'."""

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
                'base_msgs.msg.PNCMap')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pnc_map
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pnc_map
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pnc_map
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pnc_map
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pnc_map

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from visualization_msgs.msg import Marker
            if Marker.__class__._TYPE_SUPPORT is None:
                Marker.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PNCMap(metaclass=Metaclass_PNCMap):
    """Message class 'PNCMap'."""

    __slots__ = [
        '_header',
        '_road_length',
        '_road_half_width',
        '_left_boundary',
        '_right_boundary',
        '_midline',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'road_length': 'double',
        'road_half_width': 'double',
        'left_boundary': 'visualization_msgs/Marker',
        'right_boundary': 'visualization_msgs/Marker',
        'midline': 'visualization_msgs/Marker',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['visualization_msgs', 'msg'], 'Marker'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['visualization_msgs', 'msg'], 'Marker'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['visualization_msgs', 'msg'], 'Marker'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.road_length = kwargs.get('road_length', float())
        self.road_half_width = kwargs.get('road_half_width', float())
        from visualization_msgs.msg import Marker
        self.left_boundary = kwargs.get('left_boundary', Marker())
        from visualization_msgs.msg import Marker
        self.right_boundary = kwargs.get('right_boundary', Marker())
        from visualization_msgs.msg import Marker
        self.midline = kwargs.get('midline', Marker())

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
        if self.road_length != other.road_length:
            return False
        if self.road_half_width != other.road_half_width:
            return False
        if self.left_boundary != other.left_boundary:
            return False
        if self.right_boundary != other.right_boundary:
            return False
        if self.midline != other.midline:
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
    def road_length(self):
        """Message field 'road_length'."""
        return self._road_length

    @road_length.setter
    def road_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'road_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'road_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._road_length = value

    @builtins.property
    def road_half_width(self):
        """Message field 'road_half_width'."""
        return self._road_half_width

    @road_half_width.setter
    def road_half_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'road_half_width' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'road_half_width' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._road_half_width = value

    @builtins.property
    def left_boundary(self):
        """Message field 'left_boundary'."""
        return self._left_boundary

    @left_boundary.setter
    def left_boundary(self, value):
        if __debug__:
            from visualization_msgs.msg import Marker
            assert \
                isinstance(value, Marker), \
                "The 'left_boundary' field must be a sub message of type 'Marker'"
        self._left_boundary = value

    @builtins.property
    def right_boundary(self):
        """Message field 'right_boundary'."""
        return self._right_boundary

    @right_boundary.setter
    def right_boundary(self, value):
        if __debug__:
            from visualization_msgs.msg import Marker
            assert \
                isinstance(value, Marker), \
                "The 'right_boundary' field must be a sub message of type 'Marker'"
        self._right_boundary = value

    @builtins.property
    def midline(self):
        """Message field 'midline'."""
        return self._midline

    @midline.setter
    def midline(self, value):
        if __debug__:
            from visualization_msgs.msg import Marker
            assert \
                isinstance(value, Marker), \
                "The 'midline' field must be a sub message of type 'Marker'"
        self._midline = value

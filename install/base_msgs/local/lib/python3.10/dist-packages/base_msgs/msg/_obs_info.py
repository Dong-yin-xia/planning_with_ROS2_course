# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:msg/ObsInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ObsInfo(type):
    """Metaclass of message 'ObsInfo'."""

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
                'base_msgs.msg.ObsInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__obs_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__obs_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__obs_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__obs_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__obs_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ObsInfo(metaclass=Metaclass_ObsInfo):
    """Message class 'ObsInfo'."""

    __slots__ = [
        '_obs_length',
        '_obs_width',
        '_l',
        '_s',
        '_s_2path',
        '_ds_dt_2path',
        '_t_in',
        '_t_out',
    ]

    _fields_and_field_types = {
        'obs_length': 'double',
        'obs_width': 'double',
        'l': 'double',
        's': 'double',
        's_2path': 'double',
        'ds_dt_2path': 'double',
        't_in': 'double',
        't_out': 'double',
    }

    SLOT_TYPES = (
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
        self.obs_length = kwargs.get('obs_length', float())
        self.obs_width = kwargs.get('obs_width', float())
        self.l = kwargs.get('l', float())
        self.s = kwargs.get('s', float())
        self.s_2path = kwargs.get('s_2path', float())
        self.ds_dt_2path = kwargs.get('ds_dt_2path', float())
        self.t_in = kwargs.get('t_in', float())
        self.t_out = kwargs.get('t_out', float())

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
        if self.obs_length != other.obs_length:
            return False
        if self.obs_width != other.obs_width:
            return False
        if self.l != other.l:
            return False
        if self.s != other.s:
            return False
        if self.s_2path != other.s_2path:
            return False
        if self.ds_dt_2path != other.ds_dt_2path:
            return False
        if self.t_in != other.t_in:
            return False
        if self.t_out != other.t_out:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def obs_length(self):
        """Message field 'obs_length'."""
        return self._obs_length

    @obs_length.setter
    def obs_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obs_length' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obs_length' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obs_length = value

    @builtins.property
    def obs_width(self):
        """Message field 'obs_width'."""
        return self._obs_width

    @obs_width.setter
    def obs_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'obs_width' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'obs_width' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._obs_width = value

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
    def t_in(self):
        """Message field 't_in'."""
        return self._t_in

    @t_in.setter
    def t_in(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't_in' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 't_in' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._t_in = value

    @builtins.property
    def t_out(self):
        """Message field 't_out'."""
        return self._t_out

    @t_out.setter
    def t_out(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 't_out' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 't_out' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._t_out = value

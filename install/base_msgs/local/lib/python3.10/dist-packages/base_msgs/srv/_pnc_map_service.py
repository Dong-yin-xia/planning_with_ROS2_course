# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:srv/PNCMapService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PNCMapService_Request(type):
    """Metaclass of message 'PNCMapService_Request'."""

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
                'base_msgs.srv.PNCMapService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__pnc_map_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__pnc_map_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__pnc_map_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__pnc_map_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__pnc_map_service__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PNCMapService_Request(metaclass=Metaclass_PNCMapService_Request):
    """Message class 'PNCMapService_Request'."""

    __slots__ = [
        '_map_type',
    ]

    _fields_and_field_types = {
        'map_type': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.map_type = kwargs.get('map_type', int())

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
        if self.map_type != other.map_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def map_type(self):
        """Message field 'map_type'."""
        return self._map_type

    @map_type.setter
    def map_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'map_type' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'map_type' field must be an integer in [-2147483648, 2147483647]"
        self._map_type = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PNCMapService_Response(type):
    """Metaclass of message 'PNCMapService_Response'."""

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
                'base_msgs.srv.PNCMapService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__pnc_map_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__pnc_map_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__pnc_map_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__pnc_map_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__pnc_map_service__response

            from base_msgs.msg import PNCMap
            if PNCMap.__class__._TYPE_SUPPORT is None:
                PNCMap.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PNCMapService_Response(metaclass=Metaclass_PNCMapService_Response):
    """Message class 'PNCMapService_Response'."""

    __slots__ = [
        '_pnc_map',
    ]

    _fields_and_field_types = {
        'pnc_map': 'base_msgs/PNCMap',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['base_msgs', 'msg'], 'PNCMap'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from base_msgs.msg import PNCMap
        self.pnc_map = kwargs.get('pnc_map', PNCMap())

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
        if self.pnc_map != other.pnc_map:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pnc_map(self):
        """Message field 'pnc_map'."""
        return self._pnc_map

    @pnc_map.setter
    def pnc_map(self, value):
        if __debug__:
            from base_msgs.msg import PNCMap
            assert \
                isinstance(value, PNCMap), \
                "The 'pnc_map' field must be a sub message of type 'PNCMap'"
        self._pnc_map = value


class Metaclass_PNCMapService(type):
    """Metaclass of service 'PNCMapService'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('base_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'base_msgs.srv.PNCMapService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__pnc_map_service

            from base_msgs.srv import _pnc_map_service
            if _pnc_map_service.Metaclass_PNCMapService_Request._TYPE_SUPPORT is None:
                _pnc_map_service.Metaclass_PNCMapService_Request.__import_type_support__()
            if _pnc_map_service.Metaclass_PNCMapService_Response._TYPE_SUPPORT is None:
                _pnc_map_service.Metaclass_PNCMapService_Response.__import_type_support__()


class PNCMapService(metaclass=Metaclass_PNCMapService):
    from base_msgs.srv._pnc_map_service import PNCMapService_Request as Request
    from base_msgs.srv._pnc_map_service import PNCMapService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

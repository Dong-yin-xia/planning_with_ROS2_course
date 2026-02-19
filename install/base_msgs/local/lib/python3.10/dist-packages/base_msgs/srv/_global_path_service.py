# generated from rosidl_generator_py/resource/_idl.py.em
# with input from base_msgs:srv/GlobalPathService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GlobalPathService_Request(type):
    """Metaclass of message 'GlobalPathService_Request'."""

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
                'base_msgs.srv.GlobalPathService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__global_path_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__global_path_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__global_path_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__global_path_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__global_path_service__request

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


class GlobalPathService_Request(metaclass=Metaclass_GlobalPathService_Request):
    """Message class 'GlobalPathService_Request'."""

    __slots__ = [
        '_global_planner_type',
        '_pnc_map',
    ]

    _fields_and_field_types = {
        'global_planner_type': 'int32',
        'pnc_map': 'base_msgs/PNCMap',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['base_msgs', 'msg'], 'PNCMap'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.global_planner_type = kwargs.get('global_planner_type', int())
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
        if self.global_planner_type != other.global_planner_type:
            return False
        if self.pnc_map != other.pnc_map:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def global_planner_type(self):
        """Message field 'global_planner_type'."""
        return self._global_planner_type

    @global_planner_type.setter
    def global_planner_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'global_planner_type' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'global_planner_type' field must be an integer in [-2147483648, 2147483647]"
        self._global_planner_type = value

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


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GlobalPathService_Response(type):
    """Metaclass of message 'GlobalPathService_Response'."""

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
                'base_msgs.srv.GlobalPathService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__global_path_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__global_path_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__global_path_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__global_path_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__global_path_service__response

            from nav_msgs.msg import Path
            if Path.__class__._TYPE_SUPPORT is None:
                Path.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GlobalPathService_Response(metaclass=Metaclass_GlobalPathService_Response):
    """Message class 'GlobalPathService_Response'."""

    __slots__ = [
        '_global_path',
    ]

    _fields_and_field_types = {
        'global_path': 'nav_msgs/Path',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Path'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from nav_msgs.msg import Path
        self.global_path = kwargs.get('global_path', Path())

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
        if self.global_path != other.global_path:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def global_path(self):
        """Message field 'global_path'."""
        return self._global_path

    @global_path.setter
    def global_path(self, value):
        if __debug__:
            from nav_msgs.msg import Path
            assert \
                isinstance(value, Path), \
                "The 'global_path' field must be a sub message of type 'Path'"
        self._global_path = value


class Metaclass_GlobalPathService(type):
    """Metaclass of service 'GlobalPathService'."""

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
                'base_msgs.srv.GlobalPathService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__global_path_service

            from base_msgs.srv import _global_path_service
            if _global_path_service.Metaclass_GlobalPathService_Request._TYPE_SUPPORT is None:
                _global_path_service.Metaclass_GlobalPathService_Request.__import_type_support__()
            if _global_path_service.Metaclass_GlobalPathService_Response._TYPE_SUPPORT is None:
                _global_path_service.Metaclass_GlobalPathService_Response.__import_type_support__()


class GlobalPathService(metaclass=Metaclass_GlobalPathService):
    from base_msgs.srv._global_path_service import GlobalPathService_Request as Request
    from base_msgs.srv._global_path_service import GlobalPathService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

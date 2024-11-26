# generated from rosidl_generator_py/resource/_idl.py.em
# with input from interfaces:srv/MoveServo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MoveServo_Request(type):
    """Metaclass of message 'MoveServo_Request'."""

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
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.srv.MoveServo_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__move_servo__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__move_servo__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__move_servo__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__move_servo__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__move_servo__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveServo_Request(metaclass=Metaclass_MoveServo_Request):
    """Message class 'MoveServo_Request'."""

    __slots__ = [
        '_port',
        '_pos',
    ]

    _fields_and_field_types = {
        'port': 'int64',
        'pos': 'int64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.port = kwargs.get('port', int())
        self.pos = kwargs.get('pos', int())

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
        if self.port != other.port:
            return False
        if self.pos != other.pos:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def port(self):
        """Message field 'port'."""
        return self._port

    @port.setter
    def port(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'port' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'port' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._port = value

    @builtins.property
    def pos(self):
        """Message field 'pos'."""
        return self._pos

    @pos.setter
    def pos(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pos' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'pos' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._pos = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_MoveServo_Response(type):
    """Metaclass of message 'MoveServo_Response'."""

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
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.srv.MoveServo_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__move_servo__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__move_servo__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__move_servo__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__move_servo__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__move_servo__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MoveServo_Response(metaclass=Metaclass_MoveServo_Response):
    """Message class 'MoveServo_Response'."""

    __slots__ = [
        '_status',
        '_status_msg',
    ]

    _fields_and_field_types = {
        'status': 'boolean',
        'status_msg': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', bool())
        self.status_msg = kwargs.get('status_msg', str())

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
        if self.status != other.status:
            return False
        if self.status_msg != other.status_msg:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'status' field must be of type 'bool'"
        self._status = value

    @builtins.property
    def status_msg(self):
        """Message field 'status_msg'."""
        return self._status_msg

    @status_msg.setter
    def status_msg(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'status_msg' field must be of type 'str'"
        self._status_msg = value


class Metaclass_MoveServo(type):
    """Metaclass of service 'MoveServo'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'interfaces.srv.MoveServo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__move_servo

            from interfaces.srv import _move_servo
            if _move_servo.Metaclass_MoveServo_Request._TYPE_SUPPORT is None:
                _move_servo.Metaclass_MoveServo_Request.__import_type_support__()
            if _move_servo.Metaclass_MoveServo_Response._TYPE_SUPPORT is None:
                _move_servo.Metaclass_MoveServo_Response.__import_type_support__()


class MoveServo(metaclass=Metaclass_MoveServo):
    from interfaces.srv._move_servo import MoveServo_Request as Request
    from interfaces.srv._move_servo import MoveServo_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

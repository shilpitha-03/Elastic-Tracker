# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/ReplanState.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'ini_state'
# Member 'path'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReplanState(type):
    """Metaclass of message 'ReplanState'."""

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
            module = import_type_support('quadrotor_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'quadrotor_msgs.msg.ReplanState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__replan_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__replan_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__replan_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__replan_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__replan_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

            from nav_msgs.msg import Odometry
            if Odometry.__class__._TYPE_SUPPORT is None:
                Odometry.__class__.__import_type_support__()

            from quadrotor_msgs.msg import OccMap3d
            if OccMap3d.__class__._TYPE_SUPPORT is None:
                OccMap3d.__class__.__import_type_support__()

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


class ReplanState(metaclass=Metaclass_ReplanState):
    """Message class 'ReplanState'."""

    __slots__ = [
        '_header',
        '_state',
        '_ini_state',
        '_target',
        '_occmap',
        '_path',
        '_replan_stamp',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'state': 'int16',
        'ini_state': 'sequence<double>',
        'target': 'nav_msgs/Odometry',
        'occmap': 'quadrotor_msgs/OccMap3d',
        'path': 'sequence<double>',
        'replan_stamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['nav_msgs', 'msg'], 'Odometry'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['quadrotor_msgs', 'msg'], 'OccMap3d'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.state = kwargs.get('state', int())
        self.ini_state = array.array('d', kwargs.get('ini_state', []))
        from nav_msgs.msg import Odometry
        self.target = kwargs.get('target', Odometry())
        from quadrotor_msgs.msg import OccMap3d
        self.occmap = kwargs.get('occmap', OccMap3d())
        self.path = array.array('d', kwargs.get('path', []))
        from builtin_interfaces.msg import Time
        self.replan_stamp = kwargs.get('replan_stamp', Time())

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
        if self.state != other.state:
            return False
        if self.ini_state != other.ini_state:
            return False
        if self.target != other.target:
            return False
        if self.occmap != other.occmap:
            return False
        if self.path != other.path:
            return False
        if self.replan_stamp != other.replan_stamp:
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
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'state' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'state' field must be an integer in [-32768, 32767]"
        self._state = value

    @builtins.property
    def ini_state(self):
        """Message field 'ini_state'."""
        return self._ini_state

    @ini_state.setter
    def ini_state(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'ini_state' array.array() must have the type code of 'd'"
            self._ini_state = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'ini_state' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._ini_state = array.array('d', value)

    @builtins.property
    def target(self):
        """Message field 'target'."""
        return self._target

    @target.setter
    def target(self, value):
        if __debug__:
            from nav_msgs.msg import Odometry
            assert \
                isinstance(value, Odometry), \
                "The 'target' field must be a sub message of type 'Odometry'"
        self._target = value

    @builtins.property
    def occmap(self):
        """Message field 'occmap'."""
        return self._occmap

    @occmap.setter
    def occmap(self, value):
        if __debug__:
            from quadrotor_msgs.msg import OccMap3d
            assert \
                isinstance(value, OccMap3d), \
                "The 'occmap' field must be a sub message of type 'OccMap3d'"
        self._occmap = value

    @builtins.property
    def path(self):
        """Message field 'path'."""
        return self._path

    @path.setter
    def path(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'path' array.array() must have the type code of 'd'"
            self._path = value
            return
        if __debug__:
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'path' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._path = array.array('d', value)

    @builtins.property
    def replan_stamp(self):
        """Message field 'replan_stamp'."""
        return self._replan_stamp

    @replan_stamp.setter
    def replan_stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'replan_stamp' field must be a sub message of type 'Time'"
        self._replan_stamp = value

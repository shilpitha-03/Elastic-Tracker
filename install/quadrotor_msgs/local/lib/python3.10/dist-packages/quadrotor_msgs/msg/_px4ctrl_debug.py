# generated from rosidl_generator_py/resource/_idl.py.em
# with input from quadrotor_msgs:msg/Px4ctrlDebug.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Px4ctrlDebug(type):
    """Metaclass of message 'Px4ctrlDebug'."""

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
                'quadrotor_msgs.msg.Px4ctrlDebug')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__px4ctrl_debug
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__px4ctrl_debug
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__px4ctrl_debug
            cls._TYPE_SUPPORT = module.type_support_msg__msg__px4ctrl_debug
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__px4ctrl_debug

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


class Px4ctrlDebug(metaclass=Metaclass_Px4ctrlDebug):
    """Message class 'Px4ctrlDebug'."""

    __slots__ = [
        '_header',
        '_des_v_x',
        '_des_v_y',
        '_des_v_z',
        '_fb_a_x',
        '_fb_a_y',
        '_fb_a_z',
        '_des_a_x',
        '_des_a_y',
        '_des_a_z',
        '_des_q_x',
        '_des_q_y',
        '_des_q_z',
        '_des_q_w',
        '_des_thr',
        '_thr2acc',
        '_thr_scale_compensate',
        '_voltage',
        '_err_axisang_x',
        '_err_axisang_y',
        '_err_axisang_z',
        '_err_axisang_ang',
        '_fb_rate_x',
        '_fb_rate_y',
        '_fb_rate_z',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'des_v_x': 'double',
        'des_v_y': 'double',
        'des_v_z': 'double',
        'fb_a_x': 'double',
        'fb_a_y': 'double',
        'fb_a_z': 'double',
        'des_a_x': 'double',
        'des_a_y': 'double',
        'des_a_z': 'double',
        'des_q_x': 'double',
        'des_q_y': 'double',
        'des_q_z': 'double',
        'des_q_w': 'double',
        'des_thr': 'double',
        'thr2acc': 'double',
        'thr_scale_compensate': 'double',
        'voltage': 'double',
        'err_axisang_x': 'double',
        'err_axisang_y': 'double',
        'err_axisang_z': 'double',
        'err_axisang_ang': 'double',
        'fb_rate_x': 'double',
        'fb_rate_y': 'double',
        'fb_rate_z': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.des_v_x = kwargs.get('des_v_x', float())
        self.des_v_y = kwargs.get('des_v_y', float())
        self.des_v_z = kwargs.get('des_v_z', float())
        self.fb_a_x = kwargs.get('fb_a_x', float())
        self.fb_a_y = kwargs.get('fb_a_y', float())
        self.fb_a_z = kwargs.get('fb_a_z', float())
        self.des_a_x = kwargs.get('des_a_x', float())
        self.des_a_y = kwargs.get('des_a_y', float())
        self.des_a_z = kwargs.get('des_a_z', float())
        self.des_q_x = kwargs.get('des_q_x', float())
        self.des_q_y = kwargs.get('des_q_y', float())
        self.des_q_z = kwargs.get('des_q_z', float())
        self.des_q_w = kwargs.get('des_q_w', float())
        self.des_thr = kwargs.get('des_thr', float())
        self.thr2acc = kwargs.get('thr2acc', float())
        self.thr_scale_compensate = kwargs.get('thr_scale_compensate', float())
        self.voltage = kwargs.get('voltage', float())
        self.err_axisang_x = kwargs.get('err_axisang_x', float())
        self.err_axisang_y = kwargs.get('err_axisang_y', float())
        self.err_axisang_z = kwargs.get('err_axisang_z', float())
        self.err_axisang_ang = kwargs.get('err_axisang_ang', float())
        self.fb_rate_x = kwargs.get('fb_rate_x', float())
        self.fb_rate_y = kwargs.get('fb_rate_y', float())
        self.fb_rate_z = kwargs.get('fb_rate_z', float())

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
        if self.des_v_x != other.des_v_x:
            return False
        if self.des_v_y != other.des_v_y:
            return False
        if self.des_v_z != other.des_v_z:
            return False
        if self.fb_a_x != other.fb_a_x:
            return False
        if self.fb_a_y != other.fb_a_y:
            return False
        if self.fb_a_z != other.fb_a_z:
            return False
        if self.des_a_x != other.des_a_x:
            return False
        if self.des_a_y != other.des_a_y:
            return False
        if self.des_a_z != other.des_a_z:
            return False
        if self.des_q_x != other.des_q_x:
            return False
        if self.des_q_y != other.des_q_y:
            return False
        if self.des_q_z != other.des_q_z:
            return False
        if self.des_q_w != other.des_q_w:
            return False
        if self.des_thr != other.des_thr:
            return False
        if self.thr2acc != other.thr2acc:
            return False
        if self.thr_scale_compensate != other.thr_scale_compensate:
            return False
        if self.voltage != other.voltage:
            return False
        if self.err_axisang_x != other.err_axisang_x:
            return False
        if self.err_axisang_y != other.err_axisang_y:
            return False
        if self.err_axisang_z != other.err_axisang_z:
            return False
        if self.err_axisang_ang != other.err_axisang_ang:
            return False
        if self.fb_rate_x != other.fb_rate_x:
            return False
        if self.fb_rate_y != other.fb_rate_y:
            return False
        if self.fb_rate_z != other.fb_rate_z:
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
    def des_v_x(self):
        """Message field 'des_v_x'."""
        return self._des_v_x

    @des_v_x.setter
    def des_v_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_v_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_v_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_v_x = value

    @builtins.property
    def des_v_y(self):
        """Message field 'des_v_y'."""
        return self._des_v_y

    @des_v_y.setter
    def des_v_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_v_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_v_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_v_y = value

    @builtins.property
    def des_v_z(self):
        """Message field 'des_v_z'."""
        return self._des_v_z

    @des_v_z.setter
    def des_v_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_v_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_v_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_v_z = value

    @builtins.property
    def fb_a_x(self):
        """Message field 'fb_a_x'."""
        return self._fb_a_x

    @fb_a_x.setter
    def fb_a_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fb_a_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fb_a_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fb_a_x = value

    @builtins.property
    def fb_a_y(self):
        """Message field 'fb_a_y'."""
        return self._fb_a_y

    @fb_a_y.setter
    def fb_a_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fb_a_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fb_a_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fb_a_y = value

    @builtins.property
    def fb_a_z(self):
        """Message field 'fb_a_z'."""
        return self._fb_a_z

    @fb_a_z.setter
    def fb_a_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fb_a_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fb_a_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fb_a_z = value

    @builtins.property
    def des_a_x(self):
        """Message field 'des_a_x'."""
        return self._des_a_x

    @des_a_x.setter
    def des_a_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_a_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_a_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_a_x = value

    @builtins.property
    def des_a_y(self):
        """Message field 'des_a_y'."""
        return self._des_a_y

    @des_a_y.setter
    def des_a_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_a_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_a_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_a_y = value

    @builtins.property
    def des_a_z(self):
        """Message field 'des_a_z'."""
        return self._des_a_z

    @des_a_z.setter
    def des_a_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_a_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_a_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_a_z = value

    @builtins.property
    def des_q_x(self):
        """Message field 'des_q_x'."""
        return self._des_q_x

    @des_q_x.setter
    def des_q_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_q_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_q_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_q_x = value

    @builtins.property
    def des_q_y(self):
        """Message field 'des_q_y'."""
        return self._des_q_y

    @des_q_y.setter
    def des_q_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_q_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_q_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_q_y = value

    @builtins.property
    def des_q_z(self):
        """Message field 'des_q_z'."""
        return self._des_q_z

    @des_q_z.setter
    def des_q_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_q_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_q_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_q_z = value

    @builtins.property
    def des_q_w(self):
        """Message field 'des_q_w'."""
        return self._des_q_w

    @des_q_w.setter
    def des_q_w(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_q_w' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_q_w' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_q_w = value

    @builtins.property
    def des_thr(self):
        """Message field 'des_thr'."""
        return self._des_thr

    @des_thr.setter
    def des_thr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'des_thr' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'des_thr' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._des_thr = value

    @builtins.property
    def thr2acc(self):
        """Message field 'thr2acc'."""
        return self._thr2acc

    @thr2acc.setter
    def thr2acc(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thr2acc' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thr2acc' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thr2acc = value

    @builtins.property
    def thr_scale_compensate(self):
        """Message field 'thr_scale_compensate'."""
        return self._thr_scale_compensate

    @thr_scale_compensate.setter
    def thr_scale_compensate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thr_scale_compensate' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'thr_scale_compensate' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._thr_scale_compensate = value

    @builtins.property
    def voltage(self):
        """Message field 'voltage'."""
        return self._voltage

    @voltage.setter
    def voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'voltage' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'voltage' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._voltage = value

    @builtins.property
    def err_axisang_x(self):
        """Message field 'err_axisang_x'."""
        return self._err_axisang_x

    @err_axisang_x.setter
    def err_axisang_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_axisang_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'err_axisang_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._err_axisang_x = value

    @builtins.property
    def err_axisang_y(self):
        """Message field 'err_axisang_y'."""
        return self._err_axisang_y

    @err_axisang_y.setter
    def err_axisang_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_axisang_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'err_axisang_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._err_axisang_y = value

    @builtins.property
    def err_axisang_z(self):
        """Message field 'err_axisang_z'."""
        return self._err_axisang_z

    @err_axisang_z.setter
    def err_axisang_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_axisang_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'err_axisang_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._err_axisang_z = value

    @builtins.property
    def err_axisang_ang(self):
        """Message field 'err_axisang_ang'."""
        return self._err_axisang_ang

    @err_axisang_ang.setter
    def err_axisang_ang(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'err_axisang_ang' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'err_axisang_ang' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._err_axisang_ang = value

    @builtins.property
    def fb_rate_x(self):
        """Message field 'fb_rate_x'."""
        return self._fb_rate_x

    @fb_rate_x.setter
    def fb_rate_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fb_rate_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fb_rate_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fb_rate_x = value

    @builtins.property
    def fb_rate_y(self):
        """Message field 'fb_rate_y'."""
        return self._fb_rate_y

    @fb_rate_y.setter
    def fb_rate_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fb_rate_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fb_rate_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fb_rate_y = value

    @builtins.property
    def fb_rate_z(self):
        """Message field 'fb_rate_z'."""
        return self._fb_rate_z

    @fb_rate_z.setter
    def fb_rate_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fb_rate_z' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'fb_rate_z' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._fb_rate_z = value

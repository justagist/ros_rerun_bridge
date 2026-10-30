"""Checks on message fields named by dotted paths in a config, e.g. `twist.twist.linear.x`."""

from __future__ import annotations

import re

# Not octet (`byte`): its values are `bytes` objects, and numeric bytes are uint8 by convention
_NUMERIC = {
    "boolean",
    "float",
    "double",
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
}
# Element type of an unbounded or bounded sequence, e.g. "sequence<double, 3>", or of a fixed array, e.g. "double[9]".
# The element type keeps a bounded string's bound, e.g. "sequence<string<22>>".
_ARRAY = re.compile(r"sequence<(\w+(?:<=?\d+>)?)(?:, \d+)?>|(\w+(?:<=?\d+>)?)\[\d+\]")
# A string or wide string, unbounded or bounded, e.g. "string<10>"
_STRING = re.compile(r"w?string(?:<=?\d+>?)?")


def field_type(msg_type: type, field: str) -> str:
    """Return the ROS type of a field named by a dotted path, raising a ValueError if the message has no such field.

    Args:
        msg_type: ROS message class.
        field: Dotted path, e.g. `twist.twist.linear.x`.

    Returns:
        The field's ROS type, e.g. `double`, `sequence<double>` or `std_msgs/Header`.
    """
    cls = msg_type
    names = field.split(".")
    for i, name in enumerate(names):
        if not hasattr(cls, "get_fields_and_field_types"):
            raise ValueError(f"field '{field}': '{'.'.join(names[:i])}' is not a message")
        ros_type = cls.get_fields_and_field_types().get(name)
        if ros_type is None:
            raise ValueError(f"field '{field}': {cls.__name__} has no field '{name}'")
        # A default message holds every nested message, so the next level's class comes from it
        cls = type(getattr(cls(), name))
    return ros_type


def check_field(msg_type: type, field: str) -> str:
    """Raise a ValueError unless a dotted field path names a number, or an array of numbers, in a message type.

    Args:
        msg_type: ROS message class.
        field: Dotted path, e.g. `twist.twist.linear.x`.

    Returns:
        The field's ROS type, e.g. `double` or `sequence<double>`.
    """
    ros_type = field_type(msg_type, field)
    array = _ARRAY.fullmatch(ros_type)
    if (array.group(1) or array.group(2) if array else ros_type) not in _NUMERIC:
        raise ValueError(f"field '{field}' is {ros_type}, not a number or an array of numbers")
    return ros_type


def is_single_value(field_type: str) -> bool:
    """Whether a ROS field type is one number, bool or string: not an array or a message."""
    return field_type in _NUMERIC or _STRING.fullmatch(field_type) is not None


def is_string(field_type: str) -> bool:
    """Whether a ROS field type is a string or wide string."""
    return _STRING.fullmatch(field_type) is not None


def is_string_array(field_type: str) -> bool:
    """Whether a ROS field type is an array or sequence of strings."""
    array = _ARRAY.fullmatch(field_type)
    return array is not None and is_string(array.group(1) or array.group(2))


def is_array(field_type: str) -> bool:
    """Whether a ROS field type, as `check_field` returns it, is an array or sequence."""
    return _ARRAY.fullmatch(field_type) is not None

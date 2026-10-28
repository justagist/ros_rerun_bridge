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
# Element type of an unbounded or bounded sequence, e.g. "sequence<double, 3>", or of a fixed array, e.g. "double[9]"
_ARRAY = re.compile(r"sequence<(\w+)(?:, \d+)?>|(\w+)\[\d+\]")


def check_field(msg_type: type, field: str) -> str:
    """Raise a ValueError unless a dotted field path names a number, or an array of numbers, in a message type.

    Args:
        msg_type: ROS message class.
        field: Dotted path, e.g. `twist.twist.linear.x`.

    Returns:
        The field's ROS type, e.g. `double` or `sequence<double>`.
    """
    cls = msg_type
    names = field.split(".")
    for i, name in enumerate(names):
        if not hasattr(cls, "get_fields_and_field_types"):
            raise ValueError(f"field '{field}': '{'.'.join(names[:i])}' is not a message")
        field_type = cls.get_fields_and_field_types().get(name)
        if field_type is None:
            raise ValueError(f"field '{field}': {cls.__name__} has no field '{name}'")
        # A default message holds every nested message, so the next level's class comes from it
        cls = type(getattr(cls(), name))
    array = _ARRAY.fullmatch(field_type)
    if (array.group(1) or array.group(2) if array else field_type) not in _NUMERIC:
        raise ValueError(f"field '{field}' is {field_type}, not a number or an array of numbers")
    return field_type


def is_array(field_type: str) -> bool:
    """Whether a ROS field type, as `check_field` returns it, is an array or sequence."""
    return _ARRAY.fullmatch(field_type) is not None

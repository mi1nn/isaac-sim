from typing import Any, Dict, List, Mapping, Set, Tuple

from isaaclab.utils.dict import *  # noqa: F403
from isaaclab.utils.dict import string_to_slice


def replace_slices_with_strings(data: Dict[Any, Any]) -> Dict[Any, Any]:
    """Replace slice objects with their string representations in a dictionary.

    Args:
        data: The dictionary to process.

    Returns:
        The dictionary with slice objects replaced by their string representations.
    """
    if isinstance(data, Mapping):
        return {k: replace_slices_with_strings(v) for k, v in data.items()}
    elif isinstance(data, Tuple):
        return tuple(replace_slices_with_strings(v) for v in data)
    elif isinstance(data, List):
        return [replace_slices_with_strings(v) for v in data]
    elif isinstance(data, Set):
        return {replace_slices_with_strings(v) for v in data}
    elif isinstance(data, slice):
        return f"slice({data.start},{data.stop},{data.step})"
    else:
        return data


def replace_strings_with_slices(data: Dict[Any, Any]) -> Dict[Any, Any]:
    """Replace string representations of slices with slice objects in a dictionary.

    Args:
        data: The dictionary to process.

    Returns:
        The dictionary with string representations of slices replaced by slice objects.
    """
    if isinstance(data, Mapping):
        return {k: replace_strings_with_slices(v) for k, v in data.items()}
    elif isinstance(data, Tuple):
        return tuple(replace_strings_with_slices(v) for v in data)
    elif isinstance(data, List):
        return [replace_strings_with_slices(v) for v in data]
    elif isinstance(data, Set):
        return {replace_strings_with_slices(v) for v in data}
    elif isinstance(data, str) and data.startswith("slice("):
        return string_to_slice(data)
    else:
        return data

try:
    from itertools import zip_longest
except:
    from itertools import izip_longest as zip_longest
import re


cached_paths = {}
cached_result = {}


def is_leaf(original_path):
    if hasattr(cached_result, original_path):
        return cached_result[original_path]

    path = original_path[1:].split('/')

    def _is_leaf(cached, path):
        if not isinstance(path, list):
            raise ValueError
        if len(path) == 0:
            raise ValueError
        if len(path) == 1:
            if path[0] in cached:
                return False
            else:
                return True

        parent = path[0]
        child = path[1:]
        if parent not in cached:
            cached[parent] = {}
        return _is_leaf(cached[parent], child)

    result = _is_leaf(cached_paths, path)
    if result is False:
        cached_result[original_path] = result
    return result


def filter_diagnostics_status_list(status_list, blacklist,
                                   blacklist_messages=None):
    """Filter list of DiagnosticStatus.

    Parameters
    ----------
    status_list : List[DiagnosticStatus]
        List of DiagnosticStatus.
    blacklist : List[str] or List[re.Pattern]
        List of blacklist.
        The path of the name contained in this list is ignored.
    blacklist_messages : List[str] or List[re.Pattern]
        List of blacklist for message.
        This has a one-to-one correspondence with blacklist.

    Returns
    -------
    filtered_status : List[DiagnosticStatus]
        List of filtered diagnostics status.
    """
    blacklist_messages = blacklist_messages or []
    filtered_status = []
    for s in status_list:
        ns = s.name
        if is_leaf(ns) is False:
            continue
        matched = False
        for bn, message in zip_longest(
                blacklist, blacklist_messages):
            if re.match(bn, ns):
                if message is None or re.match(message, s.message):
                    matched = True
                    break
        if matched is True:
            continue
        filtered_status.append(s)
    return filtered_status

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


def filter_diagnostics_status_list(status_list, blacklist):
    """Filter list of DiagnosticStatus.

    Parameters
    ----------
    status_list : diagnostic_msgs.msg._DiagnosticStatus.DiagnosticStatus
        List of DiagnosticStatus.
    black_list : List[str]
        List of blacklist.
        The path of the name contained in this list is ignored.

    Returns
    -------
    ret.values() : Dict[str, DiagnosticStatus]
        Key is namespace and value is status.
    """
    ret = {}
    for s in status_list:
        ns = s.name
        if is_leaf(ns) is False:
            continue
        if any(filter(lambda n: n in ns, blacklist)):
            continue
        ret[ns] = s
    return ret.values()

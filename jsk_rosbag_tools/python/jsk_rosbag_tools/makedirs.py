import os
import sys


PY3 = (sys.version_info[0] == 3)
PY2 = not PY3


def makedirs(name, mode=0o777, exist_ok=True):
    """An wrapper of os.makedirs that accepts exist_ok.

    Parameters
    ----------
    name : str
        path of directory
    exist_ok : bool
        if True, accepts the existence of the directory.

    Examples
    --------
    >>> from eos import makedirs
    >>> makedirs('/tmp/result_directory')
    """
    name = str(name)
    if PY2:
        try:
            os.makedirs(name, mode)
        except OSError:
            if not (exist_ok and os.path.isdir(name)):
                raise OSError('Directory {} already exists'.format(name))
    else:
        os.makedirs(name, mode, exist_ok=exist_ok)

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import tempfile


def percol_select(candidates):
    """Return items selected by percol."""
    filename = tempfile.mktemp()
    with open(filename, 'w') as f:
        f.write('\n'.join(candidates))
    output = subprocess.check_output(['percol', filename])
    return output.strip().splitlines()

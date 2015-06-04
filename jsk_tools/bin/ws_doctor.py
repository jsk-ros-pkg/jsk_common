#!/usr/bin/env python
from jsk_tools.sanity_lib import (checkWorkspace, checkROSPackagePath)

if __name__ == "__main__":
    checkROSPackagePath()
    checkWorkspace()

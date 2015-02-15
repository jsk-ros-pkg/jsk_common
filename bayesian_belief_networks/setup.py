#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup, get_global_python_destination
import sys
import os.path

sys_version=""
if sys.version_info.major == 2:
    sys_version = "2.7"
else:
    sys_version = "3"

if os.path.isfile("/etc/debian_version"):
    target_dir = 'lib/python'+ sys_version +'/dist-packages'
else:
    target_dir = 'lib/python'+ sys_version +'/site-packages'


d = generate_distutils_setup(
    packages=['bayesian','bayesian_belief_networks'],
    package_dir={'': target_dir,'bayesian_belief_networks':'src'}
    )

setup(**d)

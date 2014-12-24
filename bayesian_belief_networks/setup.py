#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup, get_global_python_destination
import sys

target_dir = 'lib/python'+ str(sys.version[:3]) +'/dist-packages'

d = generate_distutils_setup(
    packages=['bayesian','bayesian_belief_networks'],
    package_dir={'': target_dir,'bayesian_belief_networks':'src'}
    )

setup(**d)

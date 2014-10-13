#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup, get_global_python_destination

d = generate_distutils_setup(
    packages=['bayesian','bayesian_belief_networks'],
    package_dir={'': get_global_python_destination(),'bayesian_belief_networks':'src'}
)

setup(**d)

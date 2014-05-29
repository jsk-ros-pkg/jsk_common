#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bayesian','bayesian_belief_networks'],
    package_dir={'': 'lib/python2.7/dist-packages','bayesian_belief_networks':'src'}
)

setup(**d)

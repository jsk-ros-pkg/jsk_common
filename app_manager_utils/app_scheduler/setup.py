from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages
from setuptools import setup


d = generate_distutils_setup(
    packages=find_packages('src'),
    package_dir={'': 'src'},
)

setup(**d)

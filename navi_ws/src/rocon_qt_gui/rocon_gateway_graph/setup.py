#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_gateway_graph'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_gateway_graph'],
)

setup(**d)

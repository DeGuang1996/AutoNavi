#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_qt_master_info'],
    package_dir={'': 'src'},
    #scripts=['scripts/concert_teleop_app'],
)
setup(**d)

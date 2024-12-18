#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['lfd_storage', 'lfd_camera', 'lfd_program'],
    package_dir={'lfd_storage': 'src/lfd_storage',
                 'lfd_program': 'src/lfd_program',
                 'lfd_camera': 'src/lfd_camera'}
)

setup(**d)

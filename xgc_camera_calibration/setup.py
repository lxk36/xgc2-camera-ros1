#!/usr/bin/env python3

# Catkin on Ubuntu 20.04 deliberately passes Debian's --install-layout flag.
# A user-local modern setuptools can shadow the patched stdlib distutils and
# remove that flag, so make setup.py deterministic for source and release
# builds alike.
try:
    import _distutils_hack

    _distutils_hack.remove_shim()
except ImportError:
    pass

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


setup(
    **generate_distutils_setup(
        packages=["xgc_camera_calibration"],
        package_dir={"": "src"},
    )
)

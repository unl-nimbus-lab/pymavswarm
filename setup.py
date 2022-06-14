# pymavswarm is an interface for swarm control and interaction
# Copyright (C) 2022  Evan Palmer

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import setuptools
from setuptools import setup

VERSION = "0.0.5"

with open("README.md") as f:
    long_description = f.read()

setup(
    name="pymavswarm",
    version=VERSION,
    zip_safe=True,
    description="Python library used to communicate with robotic swarms using MAVLink",
    long_description_type="text/markdown",
    long_description=long_description,
    url="https://github.com/unl-nimbus-lab/pymavswarm",
    author="Evan Palmer",
    author_email="evanp922@gmail.com",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
        "Programming Language :: Python",
        "Topic :: Scientific/Engineering",
    ],
    license="GPLv3",
    packages=setuptools.find_packages(),
    setup_requires=["setuptools", "wheel"],
    install_requires=["pymavlink>=2.3.3", "pyserial>=3.0"],
)

# encoding=utf-8
from __future__ import print_function
import sys

PYTHON_VERSION = sys.version_info[:2]
if (2, 7) != PYTHON_VERSION < (3, 5):
    print("This mycobot version requires Python2.7, 3.5 or later.")
    sys.exit(1)

import setuptools
import textwrap
import pymycobot

try:
    long_description = (
        open("README.md", encoding="utf-8").read()
        + open("docs/README.md", encoding="utf-8").read()
    )
except:
    long_description = textwrap.dedent(
        """\
        # This is Python API for myCobot

        This is a python API for serial communication with mycobot and controlling it.

        [![home](./f3-min2.jpg)](https://www.elephantrobotics.com/en/myCobot-en/)

        ## Installation

        **Notes**:

        > Make sure that `Atom` is flashed into the top Atom, `Transponder` is flashed into the base Basic. <br>
        > The firmware `Atom` and `Transponder` download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)<br>
        > You also can use myStudio to flash them, myStudio address: [https://github.com/elephantrobotics/myStudio/releases](https://github.com/elephantrobotics/myStudio/releases)

        ### Pip

        ```bash
        pip install pymycobot --upgrade
        ```

        ### Source code

        ```bash
        git clone https://github.com/elephantrobotics/pymycobot.git <your-path>
        cd <your-path>/pymycobot
        # Install
        [sudo] python2 setup.py install
        # or
        [sudo] python3 setup.py install
        ```

        ## Usage:

        ```python
        from pymycobot import MyCobot, Angle, Coord
        from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.
        ```

        The [`demo`](./demo) directory stores some test case files.

        You can find out which interfaces pymycobot provides in `pymycobot/README.md`.

        Please go to [here](./docs/README.md).
        """
    )

setuptools.setup(
    name="pymycobot",
    version=pymycobot.__version__,
    author=pymycobot.__author__,
    author_email=pymycobot.__email__,
    description="Python API for serial communication of MyCobot.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=pymycobot.__git_url__,
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=["pyserial"],
    python_requires=">=2.7, !=3.0.*, !=3.1.*, !=3.2.*, !=3.3.*, != 3.4.*",
)

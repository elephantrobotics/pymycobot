import sys

VERSION = "2.3.3"

PYTHON_VERSION = sys.version_info[:2]
if (2, 7) != PYTHON_VERSION < (3, 5):
    print("This mycobot version requires Python2.7, 3.5 or later.")
    sys.exit(1)

import setuptools

if PYTHON_VERSION == (2, 7):
    long_description = ""
else:
    long_description = open("pymycobot/README.md").read()

setuptools.setup(
    name="pymycobot",
    version=VERSION,
    author="Zachary Zhang",
    author_email="lijun.zhang@elephantrobotics.com",
    description="Python API for serial communication of MyCobot.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/elephantrobotics/pymycobot",
    packages=setuptools.find_packages(),
    classifiers=[
        'Programming Language :: Python :: 2.7',
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=['pyserial'],
    python_requires='>=2.7, !=3.0.*, !=3.1.*, !=3.2.*, !=3.3.*, != 3.4.*',
)

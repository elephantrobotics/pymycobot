import sys

if (2, 7) != sys.version_info[:2] < (3, 5):
    print("This mycobot version requires Python2.7, 3.5 or later.")
    sys.exit(1)

import setuptools

version  = "2.1.2"

if sys.version_info[:2] == (2, 7):
    long_description = ""
else:
    long_description = open("pymycobot/README.md").read()

change_log = '''
# 2021.1.25

release v2.1.2
    - refactor pymycobot
    - add Error class

# 2021.1.20

`v1.0.6` fix get_coords() error.

relase v2.0.0

# 2021.1.16

Upload to server, can use `pip` to installation now.

# 2021.1.9

Fix the API problem that `is_moving()` and other methods of mycobot cannot be used.

# 2021.1.8

Python API add new methods:

- `jog_angle()`
- `jog_coord()`
- `jog_stop()`

# 2020.12.30

Adding usage documents to Python API.

# 2020.12.29

Python API supports python2.7

Modify the serial port to manual setting, support the use of window.

'''

setuptools.setup(
    name="pymycobot",
    version=version,
    author="Zachary Zhang",
    author_email="lijun.zhang@elephantrobotics.com",
    description="Python API for serial communication of MyCobot.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/elephantrobotics/pymycobot",
    packages=setuptools.find_packages(),
    classifiers=[
        'Programming Language :: Python :: 2.7',
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=['pyserial'],
    python_requires= '>=2.7, !=3.0.*, !=3.1.*, !=3.2.*, !=3.3.*, != 3.4.*',
)



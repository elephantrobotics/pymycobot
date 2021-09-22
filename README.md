# This is Python API for myCobot

![Python 2.7](https://img.shields.io/badge/Python-v2.7%5E-green?logo=python)
![Python 3](https://img.shields.io/badge/Python-v3%5E-green?logo=python)
[![pypi_version](https://img.shields.io/pypi/v/pymycobot?label=pypi)](https://pypi.org/project/pigit)

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

<!--
**Notes:**

> Now only the version is `Atom2.4` or later is supported. If you use an earlier version, please install `pymycobot 1.0.7`.

```bash
pip install pymycobot==1.0.7 --user
```
-->

### Source code

```bash
git clone https://github.com/elephantrobotics/pymycobot.git <your-path>
cd <your-path>/pymycobot
# Install
[sudo] python2 setup.py install
# or
[sudo] python3 setup.py install
```
Or the more modern form:
```
# Install
pip install .
# Uninstall
pip uninstall .

## Usage:

```python
from pymycobot import MyCobot, Angle, Coord
from pymycobot import PI_PORT, PI_BAUD # For raspberry pi version of mycobot.
```

The [`demo`](./demo) directory stores some test case files.

You can find out which interfaces pymycobot provides in `pymycobot/README.md`.

Please go to [here](./docs/README.md).

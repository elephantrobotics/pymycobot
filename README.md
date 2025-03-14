# This is Python API for ElephantRobotics product

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

```bash
# Install
pip install .
# Uninstall
pip uninstall .
```

## Usage:

```python
# for mycobot 280 machine
from pymycobot import MyCobot280  
from pymycobot import MyCobot280Socket
# for mycobot 320 machine
from pymycobot import MyCobot320
from pymycobot import MyCobot320Socket  
# for mecharm 270 machine
from pymycobot import MechArm270  
from pymycobot import MechArmSocket  
# for mypalletizer 260 machine
from pymycobot import MyPalletizer260  
from pymycobot import MyPalletizerSocket
# for ultraArm P340 machine
from pymycobot import ultraArmP340
```

The [`demo`](./demo) directory stores some test case files.

You can find out which interfaces pymycobot provides in `pymycobot/README.md`.

Please go to [here](./docs/README.md).


> Note: Version v3.6.0 differentiates interfaces by model. Starting from this version, the MyCobot class will no longer be maintained. For new usage, please refer to the document: 

![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)   ![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)

[MyCobot 280 API说明](./docs/MyCobot_280_zh.md) | [MyCobot 280 API Description](./docs/MyCobot_280_en.md)

[MyCobot 320 API说明](./docs/MyCobot_320_zh.md) | [MyCobot 320 API Description](./docs/MyCobot_320_en.md)

[MechArm 270 API说明](./docs/MechArm_270_zh.md) | [MechArm 270 API Description](./docs/MechArm_270_en.md)

[MyPalletizer 260 API说明](./docs/MyPalletizer_260_zh.md) | [MyPalletizer 260 API Description](./docs/MyPalletizer_260_en.md)

[myAGV API说明](./docs/myAGV_zh.md) | [myAGV API Description](./docs/myAGV_en.md)

[myArm_M&C API说明](./docs/myArm_M&C_zh.md) | [myArm_M&C API Description](./docs/myArm_M&C_en.md)

[ultraArm P340 API说明](./docs/ultraArm_P340_zh.md) | [ultraArm P340 API Description](./docs/ultraArm_P340_en.md)
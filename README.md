# This is Python API for myCobot

This is a python API for serial communication with mycobot and controlling it.

<!--![](./f3-min2.jpg)-->

## Installation

**Notes**:

<!-- This is the mycobot Python API package designed by Zhang Lijun([lijun.zhang@elephantrobotics.com]()) -->

> Make sure that `Atom` is flashed into the top Atom, `Transponder` is flashed into the base Basic. <br>
> The firmware `Atom` and `Transponder` download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)<br>
> You also can use myStudio to flash them, myStudio address: [https://github.com/elephantrobotics/myStudio/releases](https://github.com/elephantrobotics/myStudio/releases)

### Pip

```bash
pip install pymycobot --upgrade --user
```

**Notes:**

> If you want to use Gripper and IO, you should update your Atom to 2.5<br>
> Now only the version is `Atom2.4` or later is supported. If you use an earlier version, please install `pymycobot 1.0.7`.

```bash
pip install pymycobot==1.0.7 --user
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

The [`demo`](./demo) directory stores some test case files.

You can find out which interfaces pymycobot provides in `pymycobot/README.md`.

Please go to [here](./pymycobot/README.md).

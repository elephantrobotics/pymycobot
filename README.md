# This is Python API for myCobot

If you want to install it separately.

## Installation

**Notes**:

<!-- This is the mycobot Python API package designed by Zhang Lijun([lijun.zhang@elephantrobotics.com]()) -->

> Make sure that `Atom` is flashed into the top Atom, `Transponder` is flashed into the base Basic <br>
> The firmware `Atom` and `Transponder` download address: [https://github.com/elephantrobotics/myCobot/tree/main/Software](https://github.com/elephantrobotics/myCobot/tree/main/Software)<br>

### Pip

```bash
pip install pymycobot --upgrade
```

**Notes:**

> Now only the version after `Atom2.4` is supported. If you use an earlier version, please install `pymycobot 1.0.7`.

```bash
pip install pymycobot==1.0.7
```

### Source code

```bash
git clone https://github.com/elephantrobotics/pymycobot.git <your-path>
cd <your-path>/pymycobot
# Install
python2 setup.py install
# or
python3 setup.py install
```

**_`test/test.py` is a test file, you can find out which interfaces pymycobot provides in `pymycobot/README.md`._**

## Usage:

Please go to [here](./pymycobot/README.md).

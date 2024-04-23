
# myArm M&C Control Case

[中文](./README_ZH.md)

## Install dependencies

```shell
pip install -r requirement.txt
```

## Run the program

```shell
python main.py
```

## Program usage instructions

There are sequential requirements for opening the serial port: first open the serial port connection of myArmM, and then open the serial port connection of myArmC.

![img1](./resources/app_1.png)
![img2](./resources/app_2.png)

After both serial ports are opened, you can control the movement of myArmM by moving myArmC.

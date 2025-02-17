# myArm M&C control case

[中文](./README_ZH.md)

## Installation dependency

```shell
pip install -r requirement.txt
```

## Run the program

```shell
python main.py
```

## Program instructions

There is a sequence requirement for opening the serial port: first open the serial port connection of myArmM, then open the serial port connection of myArmC.

![img1](./resources/app_1.png)
![img2](./resources/app_2.png)

After both serial ports are opened, you can control the movement of myArmM by moving myArmC.
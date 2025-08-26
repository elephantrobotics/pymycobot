#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import socket
import typing as T
import utils
from serial import SerialException
from pymycobot import MyArmM


class Config:
    host = "127.0.0.1"              # C650 server host.
    port = 8001                     # C650 server port.
    angle_conversion = False        # Whether to roll back the angles of the robotic arm.
    execution_speed: int = 50       # The speed of the robotic arm.


gripper_angular_transformation_equations = lambda x: round((x - 0.08) / (-95.27 - 0.08) * (-123.13 + 1.23) - 1.23)


M750_limit_info = [
    (-170, 170),
    (-83, 83),
    (-90, 84),
    (-155, 153),
    (-91, 88),
    (-153, 153),
    (-118, 2)
]


def flexible_parameters(angles: list, rollback: bool = True) -> list:
    if rollback is True:
        angles[-1] = gripper_angular_transformation_equations(angles[-1])
        angles[1] *= -1
        angles[2] *= -1

    positions = []
    for i, angle in enumerate(angles):
        min_angle, max_angle = M750_limit_info[i]
        if angle < min_angle:
            positions.append(min_angle)
        elif angle > max_angle:
            positions.append(max_angle)
        else:
            positions.append(angle)

    return positions


def setup_socket_connect(host, port):
    try:
        socket_api = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket_api.connect((host, port))
    except ConnectionRefusedError:
        return None
    except OSError as e:
        print(f" # (Error) Error while connecting socket: {e}")
        exit(1)
    return socket_api


def setup_robotic_connect(comport: str) -> T.Optional[MyArmM]:
    robotic_api = None
    try:
        robotic_api = MyArmM(comport)
    except SerialException as serial_error:
        for error in serial_error.args:
            if error.startswith("could not open port"):
                print(f" # (Error) Serial port 【{comport}】 is not available.")
            else:
                print(f" # (Error) Error while connecting robotic: {serial_error}")
    return robotic_api


def processing_data(data):
    data = data.split('\n')[-1]
    angles = list(map(float, data[1:-1].split(',')))
    return angles


def main(host: str = Config.host, port: int = Config.port):
    serial_ports = utils.get_local_serial_port()
    if len(serial_ports) == 0:
        print(" # (Warn) No serial ports found. exit")
        return

    serial_comport = utils.select("# (Info) Please select a serial port to connect robotic arm:", serial_ports, 1)

    print(f" # (Info) Selected {serial_comport} to connect robotic arm")

    robotic_api = setup_robotic_connect(serial_comport)
    if robotic_api is None:
        print(" # (Error) Failed to connect robotic arm. exit")
        return

    print(f" # (Info) Connected to robotic arm at {serial_comport}")
    print(f" # (Info) Connecting to {host}:{port}...")

    while True:
        socket_api = setup_socket_connect(host, port)
        if socket_api is not None:
            break
        print(f" # (Error) Can't connect to {host}:{port}, connection retrying...")

    print(f" # (Info) Connected to {host}:{port}")
    print(f" # (Info) Start listening for changes in the angle of the robotic arm")
    print(f" # (Info) Press 【Ctrl+C】 to exit")
    while True:
        try:
            data = socket_api.recv(1024)
            angles = processing_data(data.decode('utf-8'))
            angles = flexible_parameters(angles, rollback=Config.angle_conversion)

            arm_angles: list = robotic_api.get_joints_angle() or []
            for joint_id, angle in enumerate(arm_angles, start=1):
                if angle > 200 or angle < -200:
                    print(
                        f" # (Error) The angle of joint {joint_id} exceeds the limit {angle}, "
                        f"the M750 cannot follow the movement"
                    )
                    break
            else:
                print(f" # (Info) Recv angles: {angles}")
                robotic_api.set_joints_angle(angles=angles, speed=Config.execution_speed)

        except KeyboardInterrupt:
            robotic_api.clear_recv_queue()
            print(" # (Info) Received an exit instruction")
            break

        except SerialException as serial_error:
            print(f" # (Error) Serial port error: {serial_error}")
            break

        except Exception as e:
            print(f" # (Error) {e}")
            break

    print(f" # (Info) Disconnecting from robotic arm at {serial_comport}...")
    robotic_api._serial_port.close()
    print(f" # (Info) Disconnecting from {host}:{port}...")
    socket_api.close()
    print(" # (Info) Exit")


if __name__ == "__main__":
    main()

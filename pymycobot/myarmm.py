# coding=utf-8
from __future__ import division

import threading


from pymycobot.common import ProtocolCode
from pymycobot.myarm_api import MyArmAPI


class MyArmM(MyArmAPI):

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        super(MyArmM, self).__init__(port, baudrate, timeout,debug)

    def set_joint_angle(self, joint_id, angle, speed):
        """Sets the individual joints to move to the target angle

        Args:
            joint_id (int) : 0 - 254
            angle (int) : 0 - 254
            speed (int) : 1 - 100
        """
        self._mesg(ProtocolCode.SEND_ANGLE, joint_id, [self._angle2int(angle)], speed)

    def set_joints_angle(self, angles, speed):
        """Sets all joints to move to the target angle

        Args:
            angles (list[int]):  0 - 254
            speed (int): 0 - 100
        """
        angles = list(map(self._angle2int, angles))
        return self._mesg(ProtocolCode.SEND_ANGLES, angles, speed)

    def is_robot_moving(self):
        """See if the robot is moving

        Returns:
            1: moving
            0: not moving
        """
        return self._mesg(ProtocolCode.IS_MOVING, has_reply=True)

    def stop_robot(self):
        """The robot stops moving"""
        self._mesg(ProtocolCode.STOP)

    # def is_in_position(self):
    #     """Whether the robot has reached the specified point
    #     Returns:
    #
    #     """
    #     return self._mesg(ProtocolCode.IS_IN_POSITION, reply=True)

    def set_servo_encoder(self, servo_id, encoder, speed):
        """Sets the individual motor motion to the target encoder potential value

        Args:
            servo_id: (int) 0 - 254
            encoder: (int) 0 - 4095
            speed: (int) 1 - 100

        """
        self._mesg(ProtocolCode.SET_ENCODER, servo_id, [encoder], speed)

    def set_servos_encoder(self, positions, speed):
        """Set the encoder potential value for multiple motors moving to the target

        Args:
            positions (list[int * 8]): 0 - 4095:
            speed (int): 1 - 100:
        """
        self._mesg(ProtocolCode.SET_ENCODERS, positions, speed)

    def set_servos_encoder_drag(self, encoders, speeds):
        """Set multiple servo motors with a specified speed to the target encoder potential value"""
        self.calibration_parameters(class_name=self.__class__.__name__, encoders=encoders, speeds=speeds)
        self._mesg(ProtocolCode.SET_ENCODERS_DRAG, encoders, speeds)

    def set_assist_out_io_state(self, io_number, status=1):
        """Set the auxiliary pin status

        Args:
            io_number: 1 - 6
            status: 0/1; 0: low; 1: high. default: 1

        """
        self._mesg(ProtocolCode.SET_AUXILIARY_PIN_STATUS, io_number, status)

    def get_assist_in_io_state(self, io_number):
        """Get the auxiliary pin status

        Args:
            io_number (int): 1 - 6

        Returns:
            0 or 1. 1: high 0: low

        """
        return self._mesg(ProtocolCode.GET_AUXILIARY_PIN_STATUS, io_number, has_reply=True)


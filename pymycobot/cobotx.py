# coding=utf-8
from pymycobot import MyArm
from pymycobot.common import ProtocolCode, write, read


class CobotX(MyArm):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        super().__init__(port, baudrate, timeout, debug)
        
    def set_solution_angles(self, angle, speed):
        """Set zero space deflection angle value
        
        Args:
            angle: Angle of joint 1.
            speed: 1 - 100.
        """
        return self._mesg(ProtocolCode.COBOTX_SET_SOLUTION_ANGLES, [self._angle2int(angle)], speed)
    
    def get_solution_angles(self):
        """Get zero space deflection angle value"""
        return self._mesg(ProtocolCode.COBOTX_GET_SOLUTION_ANGLES, has_reply=True)
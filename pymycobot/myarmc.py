# coding=utf-8

from __future__ import division

from pymycobot.common import ProtocolCode
from pymycobot.myarm_api import MyArmAPI


class MyArmC(MyArmAPI):

    def __init__(self, port, baudrate="1000000", timeout=0.1, debug=False):
        super(MyArmC, self).__init__(port, baudrate, timeout, debug)

    def is_tool_btn_clicked(self, mode=1):
        """get the end button status
        Args:
            1: atom
            2: gripper red button
            3: gripper blue button
            254: get all button status
        Returns:
            list[int]: 0/1, 1: press, 0: no press
        """
        return self._mesg(ProtocolCode.GET_ATOM_PRESS_STATUS, mode, has_reply=True)

    def get_joints_coord(self):
        """Get the coordinates
        Returns:
            list[float] * 6: joints angle
        """
        return self._mesg(ProtocolCode.GET_JOINTS_COORD, has_reply=True)

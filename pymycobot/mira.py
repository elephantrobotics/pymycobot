# coding: utf-8
import time

from pymycobot.common import ProtocolCode


class Mira:
    def __init__(self, port, baudrate="115200", timeout=0.1):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        import serial
        self._serial_port = serial.Serial(port, baudrate, timeout=timeout)
        
    def _read(self):
        time.sleep(0.1)
        data = None
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())  
        return data
        
    def release_all_servos(self):
        """relax all joints"""
        self._serial_port.write((ProtocolCode.RELEASE_SERVOS+ProtocolCode.END).encode())
        self._serial_port.flush()
        
    def focus_servos(self):
        """Lock all joints"""
        self._serial_port.write((ProtocolCode.LOCK_SERVOS+ProtocolCode.END).encode())
        self._serial_port.flush()
        
    def go_zero(self):
        """back to zero"""
        self._serial_port.write((ProtocolCode.BACK_ZERO+ProtocolCode.END).encode())
        self._serial_port.flush()
        
    def set_coords(self, x=None, y=None, z=None, speed=0):
        command = "g0"
        if x is not None:
            command += "x" + str(x)
        if y is not None:
            command += "y" + str(y)
        if z is not None:
            command += "z" + str(z)
        if speed > 0:
            command += "f" + str(speed)
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        
    def sleep(self, time):
        command = "g4 "+str(time) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        
    def set_mode(self, mode):
        """set mode
        
        Args:
            mode: 
                0 - Absolute Cartesian mode 
                1 - Relative Cartesian mode
            
        """
        if mode:
            self._serial_port.write((ProtocolCode.ABS_CARTESIAN + ProtocolCode.END).encode())
        else:
            self._serial_port.write((ProtocolCode.REL_CARTESIAN
                                     + ProtocolCode.END).encode())
        self._serial_port.flush()
        
    def set_gripper_state(self, state):
        """Set gripper state
        
        Args:
            state:
                0 - close
                1 - open
        """
        if state:
            self._serial_port.write((ProtocolCode.GIRPPER_OPEN + ProtocolCode.END).encode())
        else:
            self._serial_port.write((ProtocolCode.GIRPPER_CLOSE + ProtocolCode.END).encode())
        self._serial_port.flush()
        
    def get_joint_info(self):
        """Get all current node bit information"""
        self._serial_port.write((ProtocolCode.GET_CURRENT_JOINT_INFO + ProtocolCode.END).encode())
        self._serial_port.flush()
        return self._read()
        
    def get_switch_state(self):
        """Get the current state of all home switches"""
        self._serial_port.write((ProtocolCode.GET_BACK_ZERO_STATUS + ProtocolCode.END).encode())
        self._serial_port.flush()
        return self._read()
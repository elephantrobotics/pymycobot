# coding: utf-8
# from curses import pair_number
from asyncio.windows_events import NULL
from base64 import decode
from gettext import find
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
        time.sleep(1) 
        
        
    def _respone(self):
        time.sleep(0.1)
        data = None
        while True:       
            if self._serial_port.inWaiting() > 0:
                data = self._serial_port.read(self._serial_port.inWaiting())                
                if data != None:                    
                    if str(data).find("ok") > 0:
                        break

              
    def _request(self):
        time.sleep(0.1)
        data = None
        begin, end = 0, 0
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
            data = str(data)
            
            begin = ""
            begin1 = data.find("COORDS")
            begin2 = data.find("ANGLES")
            # print(begin1,begin2)
            if begin1 != -1:
                begin = begin1
            if begin2 != -1:
                begin = begin2

            end = data.find("ok")
            return data[begin:end-11]
            # return data
        
        
    def release_all_servos(self):
        """relax all joints"""
        self._serial_port.write((ProtocolCode.RELEASE_SERVOS+ProtocolCode.END).encode())
        self._serial_port.flush()
        self._respone()
        
        
    def power_on(self):
        """Lock all joints"""
        self._serial_port.write((ProtocolCode.LOCK_SERVOS+ProtocolCode.END).encode())
        self._serial_port.flush()
        self._respone()
        
        
    def go_zero(self):
        """back to zero"""
        self._serial_port.write((ProtocolCode.BACK_ZERO+ProtocolCode.END).encode())
        self._serial_port.flush()
        self._respone()
       
        
    def set_coords(self, x=None, y=None, z=None, speed=0):
        """Set all joints coords
        
        Args:
            coords: 
                x : 0 ~ 270 mm
                y : 0 ~ 270 mm
                z : 0 ~ 125 mm
            speed : 0-100 mm/s
        """  
        command = ProtocolCode.COORDS_SET
        if x is not None:
            command += " x" + str(x)
        if y is not None:
            command += " y" + str(y)
        if z is not None:
            command += " z" + str(z)
        if speed > 0:
            command += " f" + str(speed)
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        # print("command:",command)    
        self._respone()
        
        
    def sleep(self, time):
        command = ProtocolCode.SLEEP_TIME + " S" + str(time) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
        
        
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
        self._respone()
           
        
    def get_coord_info(self):
        """Get current Cartesian coordinate information"""
        self._serial_port.write((ProtocolCode.GET_CURRENT_COORD_INFO + ProtocolCode.END).encode())
        self._serial_port.flush()
        return self._request()
      
        
    def get_switch_state(self):
        """Get the current state of all home switches"""
        self._serial_port.write((ProtocolCode.GET_BACK_ZERO_STATUS + ProtocolCode.END).encode())
        self._serial_port.flush()
        print("switch_state:" + self._request())
        # return "switch_state:" + self._request()
    

    def set_init_pose(self, x=None, y=None, z=None):
        """Set the current coords to zero"""
        command = ProtocolCode.SET_JOINT
        if x is not None:
            command += " x" + str(x)
        if y is not None:
            command += " y" + str(y)
        if z is not None:
            command += " z" + str(z)
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
        

    def set_pwm(self, p=None):
        """ PWM control 
        
        Args:
            p (int) : Duty cycle 0 ~ 255; 128 means 50%
        """
        command = ProtocolCode.SET_PWM + "p" + str(p) + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
        
             
    def set_gpio_state(self, state):
        """Set gpio state
        
        Args:
            state:
                0 - close
                1 - open
        """
        if state:
            command = ProtocolCode.GPIO_ON + ProtocolCode.END
        else:
            command = ProtocolCode.GPIO_CLOSE + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        
        self._respone()
    
    
    def set_gripper_zero(self):
        command = ProtocolCode.GRIPPER_ZERO + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()


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
        self._respone()


    def set_fan_state(self, state):
        """Set fan state   
       
        Args:
            state:
                0 - close
                1 - open
        """
        if state:
            command = ProtocolCode.FAN_ON + ProtocolCode.END
        else:
            command = ProtocolCode.FAN_CLOSE + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
    
    
    def set_angle(self, id=None, angle=None, speed=0):
        """Set single angle
        
        Args:
            id : joint (1/2/3)
            
            angle : 
                1 : -170° ~ +170°
                2 : 0° ~ 90°
                3 : 0° ~ 75°
            speed : 0-100 mm/s
        """
        command = ProtocolCode.SET_ANGLE
        if id is not None:
            command += " j" + str(id)
        if angle is not None:
            command += " a" + str(angle)
        if speed > 0:
            command += " f" + str(speed)
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        # print(command)
        self._respone()
    
    
    def set_angles(self, x=None, y=None, z=None, speed=0):
        """Set all joints angles
        
        Args:
            angle: 
                x : -170° ~ +170°
                y : 0° ~ 90°
                z : 0° ~ 75°
            speed : 0-100 mm/s
        """  
        command = ProtocolCode.SET_ANGLES
        if x is not None:
            command += " x" + str(x)
        if y is not None:
            command += " y" + str(y)
        if z is not None:
            command += " z" + str(z)
        if speed > 0:
            command += " f" + str(speed)
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush() 
        # print(command)   
        self._respone()
    
    
    def get_angles_info(self):
        """Get the current joint angle information"""
        command = ProtocolCode.GET_CURRENT_ANGLE_INFO + ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        return self._request()
        
   
    def set_jog_angle(self, id=None, direction=None):
        """Set jog angle movement
        
        Args:
            id : joint(1/2/3)  
                   
            direction : 
                0 : positive
                1 : negative
        """
        command = ProtocolCode.JOG_ANGLE
        if id is not None:
            command += " j" + str(id)
        if direction is not None:
            command += " d" + str(direction)
       
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
    
    
    def set_jog_coord(self, axis=None, direction=None):
        """Set jog coord movement
        
        Args:
            axis : x/y/z
            
            direction: 
                0 : positive
                1 : negative
        """
        command = ProtocolCode.JOG_COORD
        if axis is not None:
            command += " j" + str(axis)
        if direction is not None:
            command += " d" + str(direction)
       
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
    
    
    def set_jog_stop(self):
        command = ProtocolCode.JOG_STOP
        command += ProtocolCode.END
        self._serial_port.write(command.encode())
        self._serial_port.flush()
        print(command)
        self._respone()
        
 
    def play_gcode_file(self, filename=None):
        '''Play the imported track file'''
        with open(filename) as f:
            commands = f.readlines()
            for i in commands:
                pass
        
    
    
    def pause_gcode_trajectory(self, filename=None):
        '''Pauses the track being played'''
        pass
    
    
    def resume_gcode_trajectory(self, filename=None):
        '''Resume playback of paused tracks'''
        pass
    
    
    def stop_gcode_trajectory(self, filename=None):
        '''Stop the track being played'''
        pass
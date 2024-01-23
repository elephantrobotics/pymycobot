# coding=utf-8

from __future__ import division
import time
import math
import logging
import threading
import struct

from pymycobot.log import setup_logging
from pymycobot.Interface import MyBuddyCommandGenerator
from pymycobot.common import ProtocolCode, write, read
from pymycobot.error import calibration_parameters


class MyBuddy(MyBuddyCommandGenerator):
    """MyCobot Python API Serial communication class.

    Supported methods:

        # Overall status
            Look at parent class: `CommandGenerator`.

        # MDI mode and operation
            get_radians()
            send_radians()
            sync_send_angles() *
            sync_send_coords() *
            Other look at parent class: `CommandGenerator`.

        # JOG mode and operation
            Look at parent class: `CommandGenerator`.

        # Running status and Settings
            Look at parent class: `CommandGenerator`.

        # Servo control
            Look at parent class: `CommandGenerator`.

        # Atom IO
            Look at parent class: `CommandGenerator`.

        # Basic
            Look at parent class: `CommandGenerator`.

        # Other
            wait() *
    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyBuddy, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()

    _write = write
    
    def _read(self):
        datas = b''
        data_len = -1
        real_data = b''
        check_digit = 0
        k = 0
        pre = 0
        t = time.time()
        while True and time.time() - t < 0.1:
            try:
                data = self._serial_port.read()
                k+=1
                if data_len == 1:
                    datas += data
                    if struct.pack("B", check_digit & 0xff) == data:
                        break
                elif len(datas) == 3:
                    data_len = struct.unpack("b",data)[0]
                    datas += data
                elif len(datas)>=2:
                    datas += data
                    if len(datas) == 5: 
                        check_digit += self._decode_int8(data)
                    elif len(datas)>5:
                        real_data += data
                    if real_data != b'':
                        check_digit += self._decode_int8(real_data)
                        real_data = b''
                    data_len -= 1
                elif data == b'\xfe':
                    if datas == b'':
                        datas += data
                        pre = k
                    else:
                        if k-1 == pre:
                            datas += data
                        else:
                            datas = b'\xfe'
                            pre = k  
            except:
                break
        else:
            datas = b''
        self.log.debug("_read: {}".format(datas))
        return datas
    
    def _mesg(self, genre, *args, **kwargs):
        """

        Args:
            genre: command type (Command)
            *args: other data.
                   It is converted to octal by default.
                   If the data needs to be encapsulated into hexadecimal,
                   the array is used to include them. (Data cannot be nested)
            **kwargs: support `has_reply`
                has_reply: Whether there is a return value to accept.
        """
        real_command, has_reply = super(
            MyBuddy, self)._mesg(genre, *args, **kwargs)
        with self.lock:
            self._write(self._flatten(real_command))

            if has_reply:
                data = self._read()
                res = self._process_received(data, genre, arm=12)
                if res == []:
                    return None
                if genre in [
                    ProtocolCode.ROBOT_VERSION,
                    ProtocolCode.SOFTWARE_VERSION,
                    ProtocolCode.GET_ROBOT_ID,
                    ProtocolCode.IS_POWER_ON,
                    ProtocolCode.IS_CONTROLLER_CONNECTED,
                    ProtocolCode.IS_PAUSED,  # TODO have bug: return b''
                    ProtocolCode.IS_IN_POSITION,
                    ProtocolCode.IS_MOVING,
                    ProtocolCode.IS_SERVO_ENABLE,
                    ProtocolCode.IS_ALL_SERVO_ENABLE,
                    ProtocolCode.GET_SERVO_DATA,
                    ProtocolCode.GET_DIGITAL_INPUT,
                    ProtocolCode.GET_GRIPPER_VALUE,
                    ProtocolCode.IS_GRIPPER_MOVING,
                    ProtocolCode.GET_SPEED,
                    ProtocolCode.GET_ENCODER,
                    ProtocolCode.GET_BASIC_INPUT,
                    ProtocolCode.GET_TOF_DISTANCE,
                    ProtocolCode.GET_END_TYPE,
                    ProtocolCode.GET_MOVEMENT_TYPE,
                    ProtocolCode.GET_REFERENCE_FRAME,
                    ProtocolCode.GET_FRESH_MODE,
                    ProtocolCode.SetHTSGripperTorque,
                    ProtocolCode.GetHTSGripperTorque,
                    ProtocolCode.GetGripperProtectCurrent,
                    ProtocolCode.InitGripper,
                    ProtocolCode.SET_FOUR_PIECES_ZERO
                    # ProtocolCode.GET_SERVO_CURRENTS
                ]:
                    return self._process_single(res)
                elif genre in [ProtocolCode.GET_ANGLES]:
                    return [self._int2angle(angle) for angle in res]
                elif genre in [ProtocolCode.GET_ANGLE]:
                    return self._int2angle(res[0]) if res else None
                elif genre in [ProtocolCode.GET_COORD]:
                    if real_command[5] < 4:
                        if real_command[2] == 3:
                            return self._int2angle(res[0]) if res else None
                        return self._int2coord(res[0]) if res else None
                    else:
                        return self._int2angle(res[0]) if res else None
                elif genre in [ProtocolCode.GET_ALL_BASE_COORDS, ProtocolCode.GET_COORDS, ProtocolCode.GET_TOOL_REFERENCE, ProtocolCode.GET_WORLD_REFERENCE, ProtocolCode.GET_BASE_COORDS, ProtocolCode.GET_BASE_COORD, ProtocolCode.BASE_TO_SINGLE_COORDS]:
                    if res:
                        r = [] 
                        for idx in range(3):
                            r.append(self._int2coord(res[idx]))
                        for idx in range(3, 6):
                            r.append(self._int2angle(res[idx]))
                        if len(res) == 12:
                            r1 = []
                            for idx in range(6, 9):
                                r1.append(self._int2coord(res[idx]))
                            for idx in range(9, 12):
                                r1.append(self._int2angle(res[idx]))
                            return [r, r1]
                        return r
                    else:
                        return res
                elif genre in [ProtocolCode.GET_JOINT_MAX_ANGLE, ProtocolCode.GET_JOINT_MIN_ANGLE]:
                    return self._int2coord(res[0])
                elif genre in [ProtocolCode.GET_SERVO_VOLTAGES, ProtocolCode.COLLISION]:
                    return [self._int2coord(angle) for angle in res]
                else:
                    return res
            return None

    def get_radians(self, id):
        """Get the radians of all joints

        Args: 
            id: 1/2 (L/R)
            
        Return:
            list: A list of float radians [radian1, ...]
        """
        angles = self._mesg(ProtocolCode.GET_ANGLES, id, has_reply=True)
        return [round(angle * (math.pi / 180), 3) for angle in angles]

    def send_radians(self, id, radians, speed):
        """Send the radians of all joints to robot arm

        Args:
            id: 1/2 (L/R).
            radians: a list of radian values( List[float]), length 6
            speed: (int )0 ~ 100
        """
        # calibration_parameters(len6=radians, speed=speed)
        degrees = [self._angle2int(radian * (180 / math.pi))
                   for radian in radians]
        return self._mesg(ProtocolCode.SEND_ANGLES, id, degrees, speed)

    # Basic for raspberry pi.
    def set_gpio_init_mode(self, mode = 1):
        """import RPiGPIO,Init GPIO mode
        
            Args:
                mode(int):0/1
                0 = BCM
                1 = BOAND
        """
        import RPi.GPIO as GPIO  # type: ignore
        self.gpio = GPIO
        if mode == 0:
            self.gpio.setmode(GPIO.BCM)
        else:
            self.gpio.setmode(GPIO.BOAND)

    def set_gpio_setup(self, pin_no, mode):
        """Init GPIO module, and set BCM mode. 

        Args:
            pin_no: (int)pin number 1-16.
        
                PIN_NO = GPIO.BCM:  

                |1 = G7  |  2 = G8  |  3 = G25 | 4 = G24  | 5 = G23  | 6 = G18   |

                |7 = G11 |  8 = G9  |  9 = G10 | 10 =G22  | 11 =G27  | 12 = G17  |

                |Grove0:   |  SCL0 = 13 = G3    |     SDA0 = 14 = G2    |

                |Grove1:   |  SCL1 = 15 = G6    |     SDA1 = 16 = G5    |

            mode: 
                0 - input
                    define: pull_up_down = DOWN
                1 - output
                     define: initial = HIGH
        """
        pin_no = self.base_io_to_gpio(pin_no)       
        if mode == 1:
            self.gpio.setup(pin_no, self.gpio.OUT)
        else:
            self.gpio.setup(pin_no, self.gpio.IN)
            
    def set_gpio_output(self, pin_no, v):
        """Set GPIO output value.

        Args:
            pin_no: (int)pin number 1-16.
            v: (int) 0 / 1
        """
        pin = self.base_io_to_gpio(pin_no)
        self.gpio.output(pin, v)
        
    def get_gpio_input(self, pin_no):
        """Get GPIO input value.

        Args:
            pin_no: (int)pin number 1-16.
        """
        pin = self.base_io_to_gpio(pin_no)
        return self.gpio.input(pin)
        
    def set_gpio_pwm_start(self, pin_no, freq = 0.5, dc = 0.5):
        """Set GPIO PWM value.

        Args:
            pin_no: (int)pin number 1-16.
            freq: (float) 0.0 - 1000000.0
            dc: (float) 0.0 - 100.0
        """
        pin = self.base_io_to_gpio(pin_no)
        self.pwm = self.gpio.PWM(pin, freq)
        self.pwm.start(dc)

    def set_gpio_pwm_change_freq(self, freq):
        """Reset GPIO PWM freq.

        Args:
            freq: (float) 0.0 - 1000000.0
        """
        self.pwm.ChangeFrequency(freq)

    def set_gpio_pwm_change_dc(self, dc):
        """Reset GPIO PWM DC.

        Args:
            dc: (float) 0.0 - 100.0
        """
        self.pwm.ChangeDutyCycle(dc)

    def set_gpio_pwm_stop(self):
        """Set GPIO PWM STOP OUTPUT.
        """
        self.pwm.stop()

    def set_iic_init(self, IIC_NO):
        """
            import SMBUS2

            Open IIC_NO port

            (For more use, please see
            
            pypilink: https://pypi.org/project/smbus2/,
             
            githublink: https://github.com/kplindegaard/smbus2)
            
        Args:
            IIC_NO(int) : 0/1 ,0 = iic0, 1=iic1
        """
        from smbus2 import SMBus
        # self.iic = SMBus(IIC_NO)
        return SMBus(IIC_NO)

    def base_io_to_gpio(self, pin):
        """BASE_io = GPIO.BCM:   
            1 = G7    |   7 = G11  
            2 = G8    |   8 = G9  
            3 = G25  |   9 = G10  
            4 = G24  |  10 =G22  
            5 = G23  |  11 =G27  
            6 = G18  |  12 = G17  
            GND       |  3V3  

            Grove0:
            SCL0 = 13 = G3  
            SDA0 = 14 = G2  
            5V  
            GND  

            Grove1:  
            SCL1 = 15 = G6  
            SDA1 = 16 = G5  
            5V  
            GND  
        """
        pin_dist = {1:7, 2:8, 3:25, 4:24, 5:23, 6:18, 
                    7:11, 8:9, 9:10, 10:22, 11:27, 12:17,
                    13:3, 14:2,'SCL0':3,"SDA0":2, 
                    15:6, 16:5, 'SCL0':6,"SDA0":5}
        if pin in pin_dist:
            _pin = pin
            pin = pin_dist.get(_pin)
        else:
            print('The IO definition exceeds the system support range, and the program exits automatically.')
            pin = None
        return pin

    def set_gpio_clearup(self,pin_no = None):
        """SET GPIO CLEANUP

        Args:
            pin_no :(int)pin number 1-16.
            if pin_no = None : cleanup all gpio
        """
        if pin_no:
            pin_no = self.base_io_to_gpio(pin_no)
        self.gpio.cleanup(pin_no)
   
    # Other
    def wait(self, t):
        time.sleep(t)
        return self

    def close(self):
        self._serial_port.close()   
            
    def open(self):
        self._serial_port.open()
        
    def is_open(self):
        return self._serial_port.is_open
    
    def sync_send_angles(self, id, degrees, speed, timeout=15):
        """Send the angle in synchronous state and return when the target point is reached
            
        Args:
            degrees: a list of degree values(List[float]), length 6.
            speed: (int) 0 ~ 100
            timeout: default 7s.
        """
        t = time.time()
        self.send_angles(id, degrees, speed)
        time.sleep(0.5)
        while time.time() - t < timeout:
            f = self.is_in_position(id, degrees, 0)
            if f == 1:
                break
            time.sleep(0.1)
        return self

    def sync_send_coords(self, id, coords, speed, mode=0, timeout=15):
        """Send the coord in synchronous state and return when the target point is reached
            
        Args:
            coords: a list of coord values(List[float])
            speed: (int) 0 ~ 100
            mode: (int): 0 - angular（default）, 1 - linear
            timeout: default 7s.
        """
        t = time.time()
        self.send_coords(id, coords, speed, mode)
        time.sleep(0.5)
        while time.time() - t < timeout:
            if self.is_in_position(id, coords, 1) == 1:
                break
            time.sleep(0.1)
        return self

# coding=utf-8

import time
import threading

from pymycobot.mercury_api import MercuryCommandGenerator
from pymycobot.error import calibration_parameters


class Pro630(MercuryCommandGenerator):
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(Pro630, self).__init__(debug)
        self.calibration_parameters = calibration_parameters
        import serial
        import RPi.GPIO as GPIO
        self.power_control_1 = 3
        self.power_control_2 = 4
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.power_control_1, GPIO.IN)
        GPIO.setup(self.power_control_2, GPIO.OUT)
        self._serial_port = serial.Serial()
        self._serial_port.port = port
        self._serial_port.baudrate = baudrate
        self._serial_port.timeout = timeout
        self._serial_port.rts = False
        self._serial_port.open()
        self.lock = threading.Lock()
        self.has_reply_command = []
        self.is_stop = False
        self.read_threading = threading.Thread(target=self.read_thread, daemon=True)
        self.read_threading.start()

    def open(self):
        self._serial_port.open()
        
    def close(self):
        self._serial_port.close()
        
    def power_on(self):
        import RPi.GPIO as GPIO
        GPIO.output(self.power_control_2, GPIO.HIGH)
        time.sleep(0.25)
        return super().power_on()
    
    def power_off(self):
        import RPi.GPIO as GPIO
        res = super().power_off()
        GPIO.output(self.power_control_2, GPIO.LOW)
        return res

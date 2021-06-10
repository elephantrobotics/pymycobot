import time
import serial
import logging

from pymycobot.log import setup_logging
from pymycobot.generate import MycobotCommandGenerater


class MyCobot(MycobotCommandGenerater):
    """MyCobot Python API Serial communication class.

    Supported methods:
        Look at parent class: `MycobotCommandGenerater`.
    """

    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):
        """
        Args:
            port     : port string
            baudrate : baud rate string, default '115200'
            timeout  : default 0.1
            debug    : whether show debug info
        """
        super(MyCobot, self).__init__(debug)
        self.debug = debug
        setup_logging(self.debug)
        self.log = logging.getLogger(__name__)
        self._serial_port = serial.Serial(port, baudrate, timeout=timeout)

    def _write(self, command):
        self.log.debug("_write: {}".format(command))

        self._serial_port.write(command)
        self._serial_port.flush()
        time.sleep(0.05)

    def _read(self, size=1024):
        if self._serial_port.inWaiting() > 0:
            data = self._serial_port.read(self._serial_port.inWaiting())
            self.log.debug("_read: {}".format(data))
        else:
            self.log.debug("_read: no data can be read")
            data = None
        return data

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
        real_command, has_reply = super()._mesg(genre, *args, **kwargs)
        self._write(self._flatten(real_command))

        if has_reply:
            data = self._read()
            res = self._process_received(data, genre)
            return res

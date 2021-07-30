import re
import serial.tools.list_ports


def get_port_list():
    plist = [str(x).split(" - ")[0].strip() for x in serial.tools.list_ports.comports()]
    return plist


def detect_port_of_basic():
    """Detect M5 Basic.
    Returns the serial port string of the first detected M5 Basic.
    If it is not found, it returns `None`.
    """
    type_re = re.compile(r"PID=[0-9a-zA-Z]+:([0-9a-zA-Z]+)")
    plist = [x for x in serial.tools.list_ports.comports()]
    for port in plist:
        res = type_re.findall(port[2])
        if res and res[0].lower() == "ea60":
            return port[0]
    else:
        return None

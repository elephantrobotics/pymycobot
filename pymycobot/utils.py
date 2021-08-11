from serial.tools.list_ports import comports

def get_port_list():
    return [p.device for p in comports()]

def detect_port_of_basic():
    """Detect M5 Basic.
    Returns the serial port string of the first detected M5 Basic.
    If it is not found, it returns `None`.
    """
    ports = [p.device for p in comports() if p.pid == 0xea60]
    return ports[0] if len(ports) > 0 else None

import serial
import serial.tools.list_ports
import os

if __name__ == "__main__":
    """
    result:
        /dev/cu.wlan-debug - n/a
        /dev/cu.debug-console - n/a
        /dev/cu.Bluetooth-Incoming-Port - n/a
        /dev/cu.usbserial-0213245D - CP2104 USB to UART Bridge Controller

    port='/dev/cu.usbserial-0213245D'
    """
    plist = list(serial.tools.list_ports.comports())
    idx = 1
    for port in plist:
        print("{} : {}".format(idx, port))
        idx += 1

    _in = input("\nPlease input 1 - {} to choice:".format(idx - 1))
    port = str(plist[int(_in) - 1]).split(" - ")[0].strip()
    print(port)
    with open(os.path.dirname(__file__) + "/port.txt", "w") as f:
        f.write(port + "\n")

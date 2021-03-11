import serial
import serial.tools.list_ports

if __name__ == "__main__":
    '''
    result:
        /dev/cu.wlan-debug - n/a
        /dev/cu.debug-console - n/a
        /dev/cu.Bluetooth-Incoming-Port - n/a
        /dev/cu.usbserial-0213245D - CP2104 USB to UART Bridge Controller

    port='/dev/cu.usbserial-0213245D'
    '''
    plist = list(serial.tools.list_ports.comports())
    for port in plist:
        print(port)

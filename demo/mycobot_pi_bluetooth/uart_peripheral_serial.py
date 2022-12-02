from textwrap import indent
import dbus.mainloop.glib
import dbus
import subprocess
from example_gatt_server import register_app_cb, register_app_error_cb
from example_gatt_server import Service, Characteristic
from example_advertisement import register_ad_cb, register_ad_error_cb
from example_advertisement import Advertisement
from gi.repository import GLib
import threading
import logging
import json
import time
import hashlib
import pathlib
import os
import serial
import sys
sys.path.append('.')
try:
    from gi.repository import GObject
except ImportError:
    import gobject as GObject



BLUEZ_SERVICE_NAME = 'org.bluez'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
UART_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
#UART_RX_CHARACTERISTIC_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
UART_TX_CHARACTERISTIC_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
#UART_UD_CHARACTERISTIC_UUID = '6e400004-b5a3-f393-e0a9-e50e24dcca9e'
UART_BE_CHARACTERISTIC_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
#UART_SE_CHARACTERISTIC_UUID = '6e400006-b5a3-f393-e0a9-e50e24dcca9e'
mainloop = None
get_app_command = None
global fb
fb = None
i = 0

def read():
    datas = b""
    data_len = -1
    k = 0
    pre = 0
    t = time.time()
    wait_time = 3
    # if genre == ProtocolCode.GET_SSID_PWD:
    #     time.sleep(0.1)
    #     if serial_obj.inWaiting() > 0:
    #         datas = serial_obj.read(serial_obj.inWaiting())
    #     return datas
    # elif genre == ProtocolCode.GET_ACCEI_DATA:
    #     wait_time = 1
    while True and time.time() - t < wait_time:
        data = serial_obj.read()
        print("readd:",data)
        k += 1
        if data_len == 1 and data == b"\xfa":
            datas += data
            break
        elif len(datas) == 2:
            data_len = struct.unpack("b", data)[0]
            datas += data
        elif len(datas) > 2 and data_len > 0:
            datas += data
            data_len -= 1
        elif data == b"\xfe":
            if datas == b"":
                datas += data
                pre = k
            else:
                if k - 1 == pre:
                    datas += data
                else:
                    datas = b"\xfe"
                    pre = k
    else:
        datas = b''
    return datas


class PlayCharacteristic(Characteristic):
    """
    The characteristic is used for the play page of the app.
    """

    def __init__(self, bus, index, service):
        Characteristic.__init__(self, bus, index, UART_TX_CHARACTERISTIC_UUID,
                                ['read', 'write'], service)

    def WriteValue(self, value, options):
        """
        Let client to write data to here.
        Receive notify type data to change the notify type.
        Receive command to enable and disable the control mode.
        Receive action command to control MarsCat.
        """
        print('[PLAY]recive type: {}'.format(bytearray(value)))

    def ReadValue(self, options):
        """
        Let client to read data from here.
        """
        global get_app_command
        print("[PLAY]send type:",get_app_command)
        serial_obj.flushInput()
        serial_obj.write(get_app_command)
        serial_obj.flush()
        time.sleep(0.1)
        if serial_obj.inWaiting() > 0:
            print(serial_obj.inWaiting())
            data = serial_obj.read(serial_obj.inWaiting())
        #data = read()
        print("[PLAY]send data:",data)
        return data
        #return [dbus.Byte(1),dbus.Byte(1),dbus.Byte(1),dbus.Byte(1),dbus.Byte(1),dbus.Byte(1)]

    def StartNotify(self):
        if self.notifying:
            logger.debug('[PLAY]Already notifying, nothing to do')
            return

        self.notifying = True
        #self.toggle_notification()
        print('[PLAY]start notifying')

    def StopNotify(self):
        if not self.notifying:
            logger.debug('[PLAY]Not notifying, nothing to do')
            return

        self.notifying = False
        #self.toggle_notification()
        print('[PLAY]stop notifying')

class BasicsCharacteristic(Characteristic):
    """
    The characteristic is used for the Basic page of the app.
    Read basic information and modify basic setting.
    """

    def __init__(self, bus, index, service):
        Characteristic.__init__(self, bus, index, UART_BE_CHARACTERISTIC_UUID,
                                ['write', 'read'], service)

    def WriteValue(self, value, options):
        """
        Rewrite the modified value to the file of the cat's setting.
        """
        #logger.debug('[BASICS]>>>>>>> Basic Characteristic [write]')
        global get_app_command
        get_app_command = bytes(value)
        print('[BASICS]remote: {}'.format(get_app_command))
        
        
        if get_app_command not in [b'\xfe\xfe\x02 \xfa',b'\xfe\xfe\x02#\xfa']:
            serial_obj.write(get_app_command)
            serial_obj.flush()
        print("write serial end")
        
        #logger.debug('[BASICS]modify over')

    def ReadValue(self, options):
        """
        Read cat's basic info from local file.
        the personality value to one decimal place.
        """
        v = [10,10,10,10,10,10]
        value = []
        for b in v:
            value.append(dbus.Byte(b))
        return value



class UartService(Service):

    def __init__(self, bus, index):
        Service.__init__(self, bus, index, UART_SERVICE_UUID, True)
        self.add_characteristic(PlayCharacteristic(bus, 0, self))
        #self.add_characteristic(RxCharacteristic(bus, 1, self))
        #self.add_characteristic(UpdateCharacteristic(bus, 2, self))
        self.add_characteristic(BasicsCharacteristic(bus, 1, self))
        #self.add_characteristic(StatsCharacteristic(bus, 4, self))


class Application(dbus.service.Object):

    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
        return response


class UartApplication(Application):

    def __init__(self, bus):
        Application.__init__(self, bus)
        self.add_service(UartService(bus, 0))


class UartAdvertisement(Advertisement):

    def __init__(self, bus, index):
        Advertisement.__init__(self, bus, index, 'peripheral')
        self.add_service_uuid(UART_SERVICE_UUID)
        self.add_local_name('MyCobot-Pi')
        self.include_tx_power = True


def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                               DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()
    for o, props in objects.items():
        if LE_ADVERTISING_MANAGER_IFACE in props and GATT_MANAGER_IFACE in props:
            return o
        print('Skip adapter:', o)
    return None


#@ai.functools.thread_run
def start_server():
    #logger.debug(f'Starting BLE')
    global mainloop
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    adapter = find_adapter(bus)
    if not adapter:
        print('BLE adapter not found')
        return

    service_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter), GATT_MANAGER_IFACE)
    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter),
                                LE_ADVERTISING_MANAGER_IFACE)

    app = UartApplication(bus)
    adv = UartAdvertisement(bus, 0)

    mainloop = GLib.MainLoop()

    service_manager.RegisterApplication(app.get_path(), {},
                                        reply_handler=register_app_cb,
                                        error_handler=register_app_error_cb)
    ad_manager.RegisterAdvertisement(adv.get_path(), {},
                                     reply_handler=register_ad_cb,
                                     error_handler=register_ad_error_cb)
    try:
        mainloop.run()
    except KeyboardInterrupt:
        adv.Release()


if __name__ == '__main__':
    
    serial_obj = serial.Serial("/dev/ttyAMA0",1000000)
    start_server()

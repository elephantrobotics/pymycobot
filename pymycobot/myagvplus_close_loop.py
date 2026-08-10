# coding=utf-8
import time
import threading
from pymycobot.common import MyAGVPlusCommand
from pymycobot.log import setup_logging

def crc16_modbus(data: bytes) -> bytes:
    """Calculate Modbus CRC-16"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([(crc >> 8) & 0xFF, crc & 0xFF])

class MyAGVPlusCloseLoop(object):
    def __init__(self, debug=False):
        self.log = setup_logging(debug)
        self.debug = debug
        self._lock = threading.Lock()
        self._latest_auto_report = None
        self.esp32_serial = None

    def _send_esp32_command(self, cmd: int, data: list, read_plaintext=False) -> bytes:
        if cmd not in (MyAGVPlusCommand.SET_COMMUNICATION_STATE, MyAGVPlusCommand.GET_COMMUNICATION_STATE):
            if hasattr(self, '_check_comm_state') and not self._check_comm_state():
                return None
                
        if not getattr(self, 'esp32_serial', None) or not self.esp32_serial.is_open:
            self.log.info("ESP32 serial not open")
            return None
            
        with self._lock:
            frame = bytearray([0xFE, 0xFE, 0x0B, cmd])
            frame.extend(data[:8])
            while len(frame) < 12:
                frame.append(0)
            
            crc_bytes = crc16_modbus(frame)
            frame.extend(crc_bytes)
            
            self.esp32_serial.write(frame)
            self.esp32_serial.flush()
            
            if getattr(self, 'debug', False) in (True, 1):
                hex_str = " ".join([f"{b:02X}" for b in frame])
                import datetime
                now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                print(f"{now} DEBU [pymycobot.common] _write: {hex_str}")
            else:
                self.log.info(f"Send ESP32: {frame.hex()}")
            
            if read_plaintext:
                start_time = time.time()
                buf = b""
                while time.time() - start_time < 0.5:
                    if self.esp32_serial.in_waiting:
                        c = self.esp32_serial.read(1)
                        buf += c
                        if buf.endswith(b"\n"):
                            self.log.info(f"Recv Plaintext: {buf}")
                            return buf.decode('utf-8', errors='ignore')
                self.log.info(f"Recv Plaintext Timeout: {buf}")
                return None
            
            _local_last_read = None
            start_time = time.time()
            buf = b""
            while time.time() - start_time < 2.0:
                if self.esp32_serial.in_waiting:
                    buf += self.esp32_serial.read(self.esp32_serial.in_waiting)
                    
                while b'\xfe\xfe' in buf:
                    idx = buf.find(b'\xfe\xfe')
                    buf = buf[idx:]
                    
                    if len(buf) >= 3:
                        length = buf[2]
                        frame_len = 3 + length
                        
                        if len(buf) >= frame_len:
                            full_frame = buf[:frame_len]
                            buf = buf[frame_len:]
                            
                            recv_cmd = full_frame[3]
                            recv_crc = full_frame[-2:]
                            calc_crc = crc16_modbus(full_frame[:-2])
                            
                            if recv_crc == calc_crc:
                                if getattr(self, 'debug', False) in (True, 1):
                                    hex_str = " ".join([f"{b:02X}" for b in full_frame])
                                    if hex_str != _local_last_read:
                                        import datetime
                                        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                                        print(f"{now} DEBU [pymycobot.common] _read: {hex_str}")
                                        _local_last_read = hex_str
                                    
                                if recv_cmd == cmd:
                                    return full_frame[4:-2]
                                elif recv_cmd == 0x25:
                                    self._latest_auto_report = full_frame
                                    continue
                            else:
                                buf = buf[2:]
                        else:
                            break
                    else:
                        break
                time.sleep(0.005)
                
            self.log.info(f"Recv ESP32 Timeout for cmd 0x{cmd:02X}")
            return None

    def get_auto_report_message(self):
        with self._lock:
            if getattr(self, 'esp32_serial', None) and self.esp32_serial.is_open:
                if self.esp32_serial.in_waiting > 0:
                    buf = self.esp32_serial.read_all()
                    idx = buf.rfind(b'\xfe\xfe\x1b\x25')
                    if idx != -1 and idx + 30 <= len(buf):
                        self._latest_auto_report = buf[idx:idx+30]
        
        if self._latest_auto_report and len(self._latest_auto_report) >= 30:
            resp = self._latest_auto_report[4:-2]
            
            machine_states = [0, 0, 0]
            battery_voltage1 = 0.0
            battery_voltage2 = 0.0
            gyro_data = [0]*18
            charge_state = 0
            
            if len(resp) >= 24:
                machine_states = [resp[0], resp[1], resp[2]]
                charge_state = format(resp[3], '08b')
                battery_voltage1 = resp[4] / 10.0
                battery_voltage2 = resp[5] / 10.0
                
                raw_gyro = bytes(resp[6:24])
                gyro_parsed = []
                import struct
                for i in range(0, 18, 2):
                    val = struct.unpack('>h', raw_gyro[i:i+2])[0]
                    gyro_parsed.append(round(val / 100.0, 2))
                gyro_data = gyro_parsed
            
            motor_info = self.get_motor_status() if hasattr(self, 'get_motor_status') else [0,0,0,0]
            if hasattr(self, 'get_motor_enable_status'):
                motor_enable = 0 if sum(self.get_motor_enable_status()) > 0 else 1
            else:
                motor_enable = 1
            
            return [machine_states[0], machine_states[1], machine_states[2], charge_state, battery_voltage1, battery_voltage2, gyro_data]
        
        return [0, 0, 0, '00000000', 0.0, 0.0, [0.0]*9]

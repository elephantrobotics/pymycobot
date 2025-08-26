
import socket
import json
import threading
import struct

class MercuryChassisError(Exception):
    pass

class MercuryChassis:
    """
    Mercury X1 mobile chassis car socket class
    """
    def __init__(self, ip=None):
        self.ifname = b"wlan0"
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if ip is not None:
            self.host = ip
        else:
            import fcntl
            try:
                self.host = socket.inet_ntoa(fcntl.ioctl(self.server_socket.fileno(), 0x8915, struct.pack('256s', self.ifname[:15]))[20:24])  #IP
            except:
                self.host = "127.0.0.1"
        self._sock.connect((self.host, 9999))
        self.recv = threading.Thread(target=self.check_move_end, daemon=True)
        self.recv.start()
        self.move_end = False
        
    def close(self):
        self._sock.close()
    
    def open(self):
        self._sock.connect((self.host, 9999))
        
    def check_move_end(self):
        while True:
            try:
                data = self._sock.recv(1024)
                data = json.loads(data)
                self.move_end = data
            except:
                pass
    
    # @property
    def is_move_end(self):
        """Is the movement over

        Returns:
            _type_: _description_
        """
        return self.move_end
        
    def go_straight(self, speed=0.25, exercise_duration=5):
        """Forward control

        Args:
            speed (float, optional): Movement speed. Defaults to 0.25. range 0 ~ 1
            exercise_duration (int, optional): Exercise duration. Defaults to 5s.
        """
        if speed < 0 or speed > 1:
            raise MercuryChassisError("The movement speed range is 0~1, but the received value is {}".format(speed))
        command = {"goStraight": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def go_back(self, speed=0.25, exercise_duration=5):
        """Back control

        Args:
            speed (float, optional): Movement speed. Defaults to 0.25. range 0 ~ 1
            exercise_duration (int, optional): Exercise duration. Defaults to 5s.
        """
        if speed < 0 or speed > 1:
            raise MercuryChassisError("The movement speed range is 0~1, but the received value is {}".format(speed))
        command = {"goBack": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def turn_left(self, speed=0.5, exercise_duration=5):
        """left turn control

        Args:
            speed (float, optional): Movement speed. Defaults to 0.25. range 0 ~ 1
            exercise_duration (int, optional): Exercise duration. Defaults to 5s.
        """
        if speed < 0 or speed > 1:
            raise MercuryChassisError("The movement speed range is 0~1, but the received value is {}".format(speed))
        command = {"turnLeft": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def turn_right(self, speed=0.5, exercise_duration=5):
        """_summary_

        Args:
            speed (float, optional): Movement speed. Defaults to 0.25. range 0 ~ 1
            exercise_duration (int, optional): Exercise duration. Defaults to 5s.
        """
        if speed < 0 or speed > 1:
            raise MercuryChassisError("The movement speed range is 0~1, but the received value is {}".format(speed))
        command = {"turnRight": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def stop(self):
        """stop motion"""
        command = {"stop": True}
        self._sock.sendall(json.dumps(command).encode())

    def init_position(self,position_x,position_y,orientation_z,orientation_w,covariance):
        """Set navigation starting position

        Args:
            position_x (_type_): _description_
            position_y (_type_): _description_
            orientation_z (_type_): _description_
            orientation_w (_type_): _description_
            covariance (_type_): _description_
        """
        command = {"initPosition": {"x": position_x, "y": position_y, "o_z": orientation_z, "o_w": orientation_w, "cov": covariance}}
        self._sock.sendall(json.dumps(command).encode())

    def goto_position(self,position_x,position_y,orientation_z,orientation_w):
        """Set navigation target location

        Args:
            position_x (_type_): _description_
            position_y (_type_): _description_
            orientation_z (_type_): _description_
            orientation_w (_type_): _description_
        """
        command = {"goToPosition": {"x": position_x, "y": position_y, "o_z": orientation_z, "o_w": orientation_w}}
        self._sock.sendall(json.dumps(command).encode())

    def cancel_navigation(self):
        """Cancel navigation
        """
        command = {"movebaseCancel": True}
        self._sock.sendall(json.dumps(command).encode())        

    def get_software_version(self):
        """Get the base server version number"""
        command = {"getSoftWareVersion": True}
        self._sock.sendall(json.dumps(command).encode()) 
        while True:
            if self.move_end:
                data = self.move_end.get("getSoftWareVersion", None)
                if data:
                    return self.move_end["getSoftWareVersion"]["return"]

    def get_base_ros_version(self):
        """Get the base ROS project version number"""
        command = {"getBaseROSVersion": True}
        self._sock.sendall(json.dumps(command).encode()) 
        while True:
            if self.move_end:
                data = self.move_end.get("getBaseROSVersion", None)
                if data:
                    return self.move_end["getBaseROSVersion"]["return"]

    def get_battery_state(self):
        """Get battery level
        """
        command = {"batteryState": True}
        self._sock.sendall(json.dumps(command).encode())
        while True:
            if self.move_end:
                data = self.move_end.get("batteryState", None)
                if data:
                    return self.move_end["batteryState"]["return"]
        

        
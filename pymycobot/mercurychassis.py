
import socket
import json
import threading

class MercuryChassis:
    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect(("192.168.123.167", 9000))
        self.recv = threading.Thread(target=self.check_move_end, daemon=True)
        self.recv.start()
        self.move_end = False
        
    def close(self):
        self._sock.close()
    
    def open(self):
        self._sock.connect(("192.168.123.167", 9000))
        
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
        return self.move_end
        
    def go_straight(self, speed=0.25, exercise_duration=5):
        command = {"goStraight": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def go_back(self, speed=0.25, exercise_duration=5):
        command = {"goBack": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def turn_left(self, speed=0.5, exercise_duration=5):
        command = {"turnLeft": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def turn_right(self, speed=0.5, exercise_duration=5):
        command = {"turnRight": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dumps(command).encode())
        
    def stop(self):
        command = {"stop": True}
        self._sock.sendall(json.dumps(command).encode())
        
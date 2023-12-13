
import socket
import json
import threading

class MercuryChassis:
    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect(("192.168.99.102", 9000))
        self.recv = threading.Thread(self.is_move_end, daemon=True)
        self.recv.start()
        self.move_end = False
        
    def is_move_end(self):
        while True:
            data = self._sock.recv(1024)
            data = json.load(data)
            key = data.keys()[0]
            self.move_end = key["return"]
    
    @property
    def move_end(self):
        return self.move_end
        
    def go_straight(self, speed=0.5, exercise_duration=5):
        command = {"goStraight": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dump(command))
        
    def go_back(self, speed=0.5, exercise_duration=5):
        command = {"goBack": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dump(command))
        
    def turn_left(self, speed=0.5, exercise_duration=5):
        command = {"turnLeft": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dump(command))
        
    def go_straight(self, speed=0.5, exercise_duration=5):
        command = {"turnRight": {"time": exercise_duration, "speed": speed}}
        self._sock.sendall(json.dump(command))
        
    def stop(self):
        command = {"stop": True}
        self._sock.sendall(json.dump(command))
        
# coding=utf-8
import socket
import threading
import unittest

from pymycobot.common import DataProcessor, ProtocolCode
from pymycobot.mercury_e1_client import MercuryE1Client


def build_frame(command, payload):
    frame = [0xFE, 0xFE, len(payload) + 3, command] + payload
    frame.extend(DataProcessor.crc_check(frame))
    return bytes(frame)


class MercuryE1ClientTest(unittest.TestCase):
    def setUp(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.commands = []
        self.close_after_accept = False
        self.server_thread = threading.Thread(target=self._serve)
        self.server_thread.daemon = True
        self.server_thread.start()

    def tearDown(self):
        self.server.close()
        self.server_thread.join(1)

    def _serve(self):
        connection, _ = self.server.accept()
        with connection:
            if self.close_after_accept:
                return
            first_command = connection.recv(1024)
            self.commands.append(first_command)
            angles = build_frame(ProtocolCode.GET_ANGLES, [0, 0] * 7)
            connection.sendall(angles[:5])
            connection.sendall(angles[5:])

            second_command = connection.recv(1024)
            self.commands.append(second_command)
            stale_angles = build_frame(ProtocolCode.GET_ANGLES, [0, 0] * 7)
            moving = build_frame(ProtocolCode.IS_MOVING, [0])
            connection.sendall(stale_angles + moving)

    def test_handles_fragmented_and_coalesced_tcp_frames(self):
        client = MercuryE1Client("127.0.0.1", self.port, timeout=1)
        try:
            self.assertEqual(client.get_angles(), [0.0] * 7)
            self.assertEqual(client.is_moving(), 0)
        finally:
            client.close()

        self.assertEqual(self.commands[0][3], ProtocolCode.GET_ANGLES)
        self.assertEqual(self.commands[1][3], ProtocolCode.IS_MOVING)
        for command in self.commands:
            self.assertEqual(DataProcessor.crc_check(command[:-2]), list(command[-2:]))

    def test_returns_minus_one_after_peer_disconnects(self):
        self.close_after_accept = True
        client = MercuryE1Client("127.0.0.1", self.port, timeout=1)
        try:
            self.assertEqual(client.get_angles(), -1)
        finally:
            client.close()


if __name__ == "__main__":
    unittest.main()

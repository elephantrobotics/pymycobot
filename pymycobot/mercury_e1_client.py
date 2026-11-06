# coding=utf-8
import locale
import socket
import threading

from pymycobot.e1_close_loop import E1CloseLoop
from pymycobot.mercury_e1 import MercuryE1


class MercuryE1Client(MercuryE1):
    """TCP client for an E1 controller exposing the native E1 protocol."""

    def __init__(self, ip="192.168.0.232", netport=6500, timeout=0.1, debug=False):
        E1CloseLoop.__init__(self, debug)
        self.SERVER_IP = ip
        self.SERVER_PORT = netport
        self.socket_timeout = timeout
        self.save_serial_log = False
        self._socket_closed = False
        self._socket_disconnected = False
        self.sock = self.connect_socket()

        self.read_threading = threading.Thread(target=self.read_thread, args=("socket",))
        self.read_threading.daemon = True
        self.read_threading.start()

        self.language, _ = locale.getdefaultlocale()
        if self.language not in ["zh_CN", "en_US"]:
            self.language = "en_US"
        self.max_joint, self.min_joint = 0, 0

    def connect_socket(self):
        sock = socket.create_connection((self.SERVER_IP, self.SERVER_PORT), self.socket_timeout)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return sock

    def open(self):
        """Open a TCP connection when the client has been closed."""
        if getattr(self, "sock", None) is None or self.sock.fileno() == -1:
            self._socket_closed = False
            self._socket_disconnected = False
            self.sock = self.connect_socket()
            self.read_threading = threading.Thread(target=self.read_thread, args=("socket",))
            self.read_threading.daemon = True
            self.read_threading.start()

    def close(self):
        """Close the TCP connection without affecting serial E1 instances."""
        self._socket_closed = True
        self._socket_disconnected = True
        if getattr(self, "sock", None) is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()

    def _send_command(self, genre, real_command):
        if self._socket_closed or self._socket_disconnected:
            return
        try:
            return super(MercuryE1Client, self)._send_command(genre, real_command)
        except OSError:
            self._socket_disconnected = True
            self.event.set()

#!/usr/bin/env python3
# coding: utf-8

"""myCobot 280 socket server for Arduino UNO Q.

The server listens on all IPv4 interfaces and forwards pymycobot socket
commands through Arduino UNO Q Bridge RPC.

Default TCP port:
    9000

Client example:
    MyCobot280Socket("192.168.1.218", 9000)

The TCP wire format remains the original myCobot protocol frame. The client
sends bytes such as FE FE 02 20 FA, and the server returns the myCobot response
frame as bytes.
"""

import logging
import logging.handlers
import socket
import subprocess
import traceback
from typing import List


HOST = "0.0.0.0"
PORT = 9000
BRIDGE_METHOD = "XferBridgeMsg"
BAUD_RATE = 1000000
BRIDGE_TIMEOUT_MS = 1000
BRIDGE_ERROR_FRAMES = {
    "FEFE035B01FA": "timeout",
    "FEFE035B02FA": "partial frame",
}


def get_logger(name: str) -> logging.Logger:
    """Create and configure the application logger."""
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter(
        "%(asctime)s - %(levelname)s - %(message)s"
    )

    console = logging.StreamHandler()
    console.setFormatter(formatter)

    file_handler = logging.handlers.RotatingFileHandler(
        "server.log",
        maxBytes=10 * 1024 * 1024,
        backupCount=1,
    )
    file_handler.setFormatter(formatter)

    logger.addHandler(console)
    logger.addHandler(file_handler)
    return logger


def get_network_ips() -> List[str]:
    """Return useful non-loopback IPv4 addresses."""
    try:
        result = subprocess.run(
            ["ip", "-4", "-o", "addr", "show", "scope", "global"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
    except (OSError, subprocess.SubprocessError):
        return []

    addresses = []
    ignored_prefixes = ("docker", "br-", "veth")

    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue

        interface = parts[1]
        if interface.startswith(ignored_prefixes):
            continue

        cidr = parts[3]
        ip_address = cidr.split("/", 1)[0]

        if ip_address and ip_address != "127.0.0.1":
            addresses.append(ip_address)

    return addresses


def frame_to_hex(frame: bytes) -> str:
    return frame.hex().upper()


def spaced_hex(hex_text: str) -> str:
    compact = "".join(str(hex_text).split()).upper()
    return " ".join(compact[i:i + 2] for i in range(0, len(compact), 2))


def bridge_rpc_timeout(timeout_ms: int) -> float:
    return max(2.0, timeout_ms / 1000.0 + 1.0)


class MyCobotServer:
    """Socket-to-Bridge server for myCobot 280 on Arduino UNO Q."""

    def __init__(
        self,
        host: str,
        port: int,
        baud: int = BAUD_RATE,
        timeout_ms: int = BRIDGE_TIMEOUT_MS,
    ):
        """Initialize the server.

        Args:
            host: IPv4 listening address. Use 0.0.0.0 for all interfaces.
            port: TCP listening port.
            baud: UART baud rate passed to Bridge RPC.
            timeout_ms: Bridge RPC firmware passthrough timeout in milliseconds.
        """
        self.logger = get_logger("MyCobotServer")
        self.host = host
        self.port = port
        self.baud = baud
        self.timeout_ms = timeout_ms

        self.bridge = self._load_bridge()
        self.server_socket = None

        self._open_server()

    def _load_bridge(self):
        """Load Arduino Bridge in the UNO Q Debian/App Lab environment."""
        try:
            from arduino.app_utils import Bridge
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "arduino.app_utils.Bridge is required. Run this server on "
                "Arduino UNO Q Debian/App Lab with arduino_app_bricks installed."
            ) from exc

        self.logger.info(
            "Bridge RPC loaded: %s(timeout_ms=%d, baud=%d)",
            BRIDGE_METHOD,
            self.timeout_ms,
            self.baud,
        )
        return Bridge

    def _open_server(self) -> None:
        """Create the TCP listening socket."""
        self.server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM,
        )
        self.server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)

        self.logger.info(
            "Server listening on %s:%d",
            self.host,
            self.port,
        )

        addresses = get_network_ips()
        if addresses:
            for ip_address in addresses:
                self.logger.info(
                    "Client example: MyCobot280Socket('%s', %d)",
                    ip_address,
                    self.port,
                )
        else:
            self.logger.info(
                "No usable network IP address is available yet. "
                "The server will remain listening on %s:%d.",
                self.host,
                self.port,
            )

    def connect(self) -> None:
        """Accept and process client connections continuously."""
        while True:
            conn = None

            try:
                self.logger.info("Waiting for client connection...")
                conn, addr = self.server_socket.accept()

                self.logger.info(
                    "Client connected: %s:%d",
                    addr[0],
                    addr[1],
                )

                self._handle_client(conn)

            except KeyboardInterrupt:
                self.logger.info("Server interrupted by user.")
                break
            except Exception:
                self.logger.error(
                    "Server exception:\n%s",
                    traceback.format_exc(),
                )
            finally:
                if conn is not None:
                    try:
                        conn.close()
                    except OSError:
                        pass
                    self.logger.info("Client disconnected.")

        self.close()

    def _handle_client(self, conn: socket.socket) -> None:
        """Handle one connected pymycobot socket client."""
        while True:
            try:
                data = conn.recv(1024)

                if not data:
                    return

                command = list(data)

                self.logger.info(
                    "Get command: %s",
                    [hex(value) for value in command],
                )

                if len(command) < 4:
                    self.logger.warning(
                        "Invalid command length: %d",
                        len(command),
                    )
                    continue

                genre = command[3]

                # Raspberry Pi GPIO commands from the original 280PI server
                # are intentionally not implemented on UNO Q.
                if genre in (0xAA, 0xAB, 0xAC, 0xAD):
                    self.logger.warning(
                        "GPIO command 0x%02X is not supported on UNO Q.",
                        genre,
                    )
                    continue

                response = self.xfer(data)

                self.logger.info(
                    "Return data: %s",
                    [hex(value) for value in response],
                )

                if response:
                    conn.sendall(response)

            except (ConnectionResetError, BrokenPipeError):
                return
            except OSError:
                self.logger.error(
                    "Network exception:\n%s",
                    traceback.format_exc(),
                )
                return
            except Exception:
                self.logger.error(
                    "Client processing exception:\n%s",
                    traceback.format_exc(),
                )
                return

    def xfer(self, command: bytes) -> bytes:
        """Transfer one myCobot frame through Bridge RPC."""
        frame_hex = frame_to_hex(command)
        self.logger.debug("_bridge_write: %s", spaced_hex(frame_hex))

        try:
            result = self.bridge.call(
                BRIDGE_METHOD,
                frame_hex,
                int(self.timeout_ms),
                int(self.baud),
                timeout=bridge_rpc_timeout(self.timeout_ms),
            )
        except Exception as exc:
            self.logger.error("_bridge_error: %s", exc)
            return b""

        if result is None:
            self.logger.error("_bridge_error: no response")
            return b""

        if isinstance(result, bytes):
            response_hex = result.hex().upper()
        else:
            response_hex = str(result).split("|", 1)[0].strip().upper()

        response_hex = "".join(response_hex.split())
        self.logger.debug("_bridge_read: %s", spaced_hex(response_hex))

        if response_hex in BRIDGE_ERROR_FRAMES:
            self.logger.error(
                "_bridge_error: %s raw=%s",
                BRIDGE_ERROR_FRAMES[response_hex],
                spaced_hex(response_hex),
            )

        try:
            return bytes.fromhex(response_hex)
        except ValueError:
            self.logger.error(
                "_bridge_error: invalid hex response=%s",
                response_hex,
            )
            return b""

    def close(self) -> None:
        """Close TCP resources."""
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass

        self.logger.info("Server resources closed.")


if __name__ == "__main__":
    logger = get_logger("MyCobotServer")

    logger.info("Starting myCobot 280 UNO Q Bridge socket server.")

    server = MyCobotServer(
        HOST,
        PORT,
        BAUD_RATE,
        BRIDGE_TIMEOUT_MS,
    )
    server.connect()

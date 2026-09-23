from pymycobot.common import ProtocolCode, read


class FakeSerial:
    def __init__(self, responses):
        self._responses = iter(responses)

    def read(self):
        return next(self._responses, b"")


class FakeLogger:
    def debug(self, _message):
        pass


class FakeReader:
    def __init__(self, responses):
        self._serial_port = FakeSerial(responses)
        self.log = FakeLogger()


def test_read_ignores_empty_byte_after_partial_header():
    reader = FakeReader([b"\xfe", b"\xfe", b""])

    assert read(reader, ProtocolCode.IS_IN_POSITION, timeout=0.001) == b""


def test_read_returns_complete_frame():
    reader = FakeReader([b"\xfe", b"\xfe", b"\x03", b"\x2a", b"\x00", b"\xfa"])

    assert read(reader, ProtocolCode.IS_IN_POSITION, timeout=0.001) == (
        b"\xfe\xfe\x03\x2a\x00\xfa"
    )

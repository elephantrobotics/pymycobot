import os
import sys
import time
import types
import logging
import pytest

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot.error import MyCobotPro450DataException, ultraArmP1DataException
from pymycobot.pro450client import Pro450Client
from pymycobot.ultraarm_p1_base import UltraArmP1Base
from pymycobot import utils

port = utils.get_port_list()
print(port)

detect_result = utils.detect_port_of_basic()
print(detect_result)


class UltraArmP1(UltraArmP1Base):
    def __init__(self, run_status_replies=None):
        super().__init__(debug=False)
        self.sent_commands = []
        self.recv_buffer = bytearray()
        self.run_status_replies = list(run_status_replies or [])
        self.STATUS_QUERY_INTERVAL = 0
        self.STATUS_QUERY_GRACE_PERIOD = 0

    def _send_command(self, command, clear_input=True):
        self.sent_commands.append(command)
        if command in ("M6", "M7"):
            self.recv_buffer.extend(f"{command} OK\n".encode())
        elif command == "M8":
            self.recv_buffer.extend(
                b"M8 acc:2000.000,400.000,400.000,400.000\n"
            )
        elif command == "M110 J1":
            self.recv_buffer.extend(b"M110 J1 Data:0.00\n")
        elif command == "M110 J4":
            self.recv_buffer.extend(b"M110 Data:0.00,0.00\n")
        elif command == "M110 J6":
            self.recv_buffer.extend(b"M110 Data:85 8C D5 AB\n")
        elif command == "M92":
            self.recv_buffer.extend(b"M92 OK\n")
        elif command == "M605":
            self.recv_buffer.extend(b"M605 SN:\n")
        elif command.startswith("M604 "):
            self.recv_buffer.extend(b"M604 OK\n")
        elif command.startswith("M9"):
            self.recv_buffer.extend(b"M9 OK\n")
        elif command == "M200" and self.run_status_replies:
            status = self.run_status_replies.pop(0)
            self.recv_buffer.extend(f"M200 mainmoving:{status}\n".encode())

    def _read_available_bytes(self):
        data = bytes(self.recv_buffer)
        self.recv_buffer.clear()
        return data

    def _clear_input_buffer(self):
        pass

    def _clear_serial_buffer(self):
        pass

    def _send_raw_command(self, command):
        self.sent_commands.append(command)

    def _query_error_information(self, timeout=0.3):
        return "ok"


def _pro450_profile_client(version_replies):
    client = object.__new__(Pro450Client)
    client._pro450_limit_profile = "Pro450Client"
    replies = iter(version_replies)
    client.get_system_version = lambda: next(replies)
    client._initialize_joint_limit_profile()
    return client


def test_pro450_legacy_profile_allows_j5_100_degrees():
    client = _pro450_profile_client([1.3])

    assert client._pro450_limit_profile == "Pro450Client"
    client._calibration_parameters(class_name="Pro450Client", joint_id=5, angle=100)
    with pytest.raises(MyCobotPro450DataException):
        client._calibration_parameters(class_name="Pro450Client", joint_id=5, angle=163)


def test_pro450_new_profile_uses_new_j5_boundaries_for_single_and_multiple_angles():
    client = _pro450_profile_client([1.4])

    assert client._pro450_limit_profile == "Pro450ClientNew"
    client._calibration_parameters(class_name="Pro450Client", joint_id=5, angle=-252)
    client._calibration_parameters(class_name="Pro450Client", joint_id=5, degree=72)
    client._calibration_parameters(
        class_name="Pro450Client", angles=[0, 0, 0, 0, 72, 0]
    )
    with pytest.raises(MyCobotPro450DataException):
        client._calibration_parameters(class_name="Pro450Client", joint_id=5, angle=100)
    with pytest.raises(MyCobotPro450DataException):
        client._calibration_parameters(
            class_name="Pro450Client", angles=[0, 0, 0, 0, -253, 0]
        )


def test_pro450_version_detection_retries_before_selecting_new_profile(monkeypatch):
    monkeypatch.setattr("pymycobot.pro450client.time.sleep", lambda _: None)
    client = _pro450_profile_client([-1, None, 1.4])

    assert client._pro450_limit_profile == "Pro450ClientNew"


def test_pro450_version_detection_falls_back_to_legacy_after_three_failures(monkeypatch):
    monkeypatch.setattr("pymycobot.pro450client.time.sleep", lambda _: None)
    client = _pro450_profile_client([-1, -1, -1])

    assert client._pro450_limit_profile == "Pro450Client"
    client._calibration_parameters(class_name="Pro450Client", joint_id=5, angle=100)


@pytest.fixture
def pro450_transport(monkeypatch):
    from pymycobot.common import DataProcessor

    monkeypatch.setattr(Pro450Client, "connect_socket", lambda self: None)
    monkeypatch.setattr(Pro450Client, "read_thread", lambda self, method: None)
    monkeypatch.setattr(Pro450Client, "is_moving", lambda self: pytest.fail("Unexpected status query"))
    monkeypatch.setattr("pymycobot.pro450client.time.sleep", lambda _: None)

    def connect(system_version, replies):
        sent = []
        replies = iter(replies)
        payload = bytes([system_version])

        def frame(opcode, payload):
            data = bytes([0xFE, 0xFE, len(payload) + 3, opcode]) + payload
            return data + bytes(DataProcessor.crc_check(data))

        def write(client, command, method=None):
            sent.append(bytes(command))
            opcode = command[3]
            if opcode == 0x02:
                incoming = frame(opcode, payload)
            else:
                assert opcode in (0x38, 0x39)
                response = next(replies)
                if isinstance(response, OSError):
                    raise response
                if response is None:
                    return
                incoming = frame(opcode, bytes([response]))
            frames, remaining = client._process_received(incoming)
            assert not remaining
            client.read_command.extend((reply, time.time()) for reply in frames)
            client.event.set()

        monkeypatch.setattr(Pro450Client, "_write", write)
        return Pro450Client(), sent

    return connect


@pytest.mark.parametrize("firmware,replies,j5_allowed", [
    (13, [], True), (14, [], False), (20, [1], True), (20, [2], False),
    (20, [None, 3, 2], False), (20, [None, 0, OSError()], True),
])
def test_pro450_machine_version_initialization(pro450_transport, firmware, replies, j5_allowed):
    client, sent = pro450_transport(firmware, replies)
    assert [frame[3] for frame in sent] == [0x02] + [0x39] * len(replies)
    assert all(frame == bytes.fromhex("FE FE 03 39 DE 90") for frame in sent[1:])

    def validate(angle):
        client.calibration_parameters(class_name="Pro450Client", joint_id=5, angle=angle)

    if j5_allowed:
        validate(100)
    else:
        with pytest.raises(MyCobotPro450DataException):
            validate(100)
        validate(-252)
        validate(72)


def test_pro450_machine_version_set_and_read_refresh_limits(pro450_transport, caplog, capsys):
    client, sent = pro450_transport(20, [1, 2, 0, 1, 0])
    client.sync_mode = False
    client.language = "zh_CN"
    assert client.set_machine_version(2) == 2
    assert sent[-1] == bytes.fromhex("FE FE 04 38 02 FC EF")
    assert client._pro450_limit_profile == "Pro450ClientNew"
    assert "结构版本已切换，请重新辨识机器" in caplog.text
    assert "⚠️ 结构版本已切换，请重新辨识机器" in capsys.readouterr().out
    assert all("⚠️" not in record.message for record in caplog.records)
    caplog.clear()
    assert client.set_machine_version(2) == 0
    assert client._pro450_limit_profile == "Pro450ClientNew"
    assert not caplog.records
    assert capsys.readouterr().out == ""
    assert client.get_machine_version() == 1
    assert client.get_machine_version() == -1
    assert client._pro450_limit_profile == "Pro450Client"
    for invalid in (0, 3, 1.0, True):
        with pytest.raises(MyCobotPro450DataException):
            client.set_machine_version(invalid)
    assert [frame[3] for frame in sent] == [2, 0x39, 0x38, 0x38, 0x39, 0x39]


def test_pro450_machine_version_commands_require_firmware_2(pro450_transport, caplog, capsys):
    client, sent = pro450_transport(14, [])
    client.language = "zh_CN"

    assert client.get_machine_version() == -1
    assert client.set_machine_version(1) == -1
    assert [frame[3] for frame in sent] == [0x02]
    assert caplog.text.count("需要 2.0 或更高版本") == 2
    assert capsys.readouterr().out.count("需要 2.0 或更高版本") == 2


def test_pro450_machine_version_commands_require_known_firmware(monkeypatch, caplog, capsys):
    client = object.__new__(Pro450Client)
    client._system_version = None
    client.language = "zh_CN"
    client.log = logging.getLogger(__name__)
    monkeypatch.setattr(client, "_mesg", lambda *args, **kwargs: pytest.fail("Unexpected protocol send"))

    assert client.get_machine_version() == -1
    assert "无法确认主控固件版本" in caplog.text
    assert "无法确认主控固件版本" in capsys.readouterr().out


@pytest.mark.parametrize("readback,profile", [
    ([2], "Pro450ClientNew"),
    ([None, None, None], "Pro450Client"),
])
def test_pro450_machine_version_lost_set_reply(pro450_transport, readback, profile):
    client, sent = pro450_transport(20, [2, None] + readback)
    assert client.set_machine_version(1) == -1
    assert client._pro450_limit_profile == profile
    assert [frame[3] for frame in sent] == [2, 0x39, 0x38] + [0x39] * len(readback)


def _pro450_screen_client():
    client = object.__new__(Pro450Client)
    client.SERVER_IP = "192.168.0.232"
    client._pro450_limit_profile = "Pro450Client"
    client.calibration_parameters = client._calibration_parameters
    return client


def test_pro450_screen_firmware_upload_uses_filename_version(monkeypatch, tmpdir):
    firmware = tmpdir.join("450_Screen_V2.0.4.bin")
    firmware.write_binary(b"firmware")
    captured = {}

    class Response(object):
        def json(self):
            return {"code": 200, "message": "ok", "data": "2.0.4"}

    def post(url, data, files, timeout):
        captured.update({
            "url": url,
            "data": data,
            "filename": files["content"][0],
            "content_type": files["content"][2],
            "timeout": timeout,
        })
        return Response()

    requests = types.SimpleNamespace(
        post=post,
        exceptions=types.SimpleNamespace(RequestException=Exception),
    )
    monkeypatch.setitem(sys.modules, "requests", requests)

    assert _pro450_screen_client().update_screen_firmware(
        str(firmware), timeout=30
    ) == "2.0.4"
    assert captured == {
        "url": "http://192.168.0.232:5890/testtool/firmware",
        "data": {"genre": "screen", "version": "2.0.4"},
        "filename": "450_Screen_V2.0.4.bin",
        "content_type": "application/octet-stream",
        "timeout": 30,
    }


def test_pro450_screen_firmware_rejects_nonstandard_filename(tmpdir):
    firmware = tmpdir.join("firmware-2.0.4.bin")
    firmware.write_binary(b"firmware")

    with pytest.raises(
        MyCobotPro450DataException, match="450_Screen_Vx.y.z.bin"
    ):
        _pro450_screen_client().update_screen_firmware(str(firmware))


def test_pro450_screen_firmware_rejects_missing_file():
    with pytest.raises(MyCobotPro450DataException, match="does not exist"):
        _pro450_screen_client().update_screen_firmware(
            "450_Screen_V2.0.4.bin"
        )


def test_ultraarm_p1_move_pause_sends_m6_and_returns_ok():
    arm = UltraArmP1()

    assert arm.move_pause() == "ok"
    assert arm.sent_commands == ["M6"]


def test_ultraarm_p1_move_resume_sends_m7_and_returns_ok():
    arm = UltraArmP1()

    assert arm.move_resume() == "ok"
    assert arm.sent_commands == ["M7"]


def test_ultraarm_p1_get_joint_acc_parses_all_joint_accelerations():
    arm = UltraArmP1()

    assert arm.get_joint_acc() == [2000.0, 400.0, 400.0, 400.0]
    assert arm.sent_commands == ["M8"]


def test_ultraarm_p1_set_joint_acc_sends_m9_with_joint_and_acc():
    arm = UltraArmP1()

    assert arm.set_joint_acc(1, 100) == "ok"
    assert arm.sent_commands == ["M9 J1 F100"]


def test_ultraarm_p1_get_default_sensor_initialize_parses_float_data():
    arm = UltraArmP1()

    assert arm.get_default_sensor_initialize(1) == 0.0
    assert arm.sent_commands == ["M110 J1"]


def test_ultraarm_p1_get_default_sensor_initialize_parses_float_list():
    arm = UltraArmP1()

    assert arm.get_default_sensor_initialize(4) == [0.0, 0.0]
    assert arm.sent_commands == ["M110 J4"]


def test_ultraarm_p1_get_default_sensor_initialize_parses_rfid_id():
    arm = UltraArmP1()

    assert arm.get_default_sensor_initialize(6) == "858CD5AB"
    assert arm.sent_commands == ["M110 J6"]


def test_ultraarm_p1_motion_wait_requires_three_consecutive_stop_statuses():
    arm = UltraArmP1(run_status_replies=[1, 0, 0, 1, 0, 0, 0])

    assert arm._response() == "ok"
    assert arm.sent_commands.count("M200") == 7


def test_ultraarm_p1_reports_j4_collision_in_chinese():
    arm = UltraArmP1()
    arm.language = "zh_CN"
    arm.recv_buffer.extend(b"CollisionDetectionError:4\n")

    assert arm._response() == "J4关节碰撞,请使用collision_unlock()接口解锁"


def test_ultraarm_p1_get_empty_sn_code_returns_empty_string():
    arm = UltraArmP1()

    assert arm._request_with_retry("M605", "get_sn_code") == ""
    assert arm.sent_commands == ["M605"]


def test_ultraarm_p1_accepts_eruap_sn_code_format():
    arm = UltraArmP1()

    arm.calibration_parameters(
        class_name=arm.__class__.__name__,
        sn_code="ERUAP12608000001",
    )


def test_ultraarm_p1_set_sn_code_normalizes_lowercase_prefix():
    arm = UltraArmP1()
    arm._internal_mode = True

    assert arm._internal_set_sn_code("eruap12608000001") == "ok"
    assert arm.sent_commands == ["M604 ERUAP12608000001"]


@pytest.mark.parametrize(
    "sn_code",
    ["12608000001", "ERUAP1260800000", "ERUAP1260800000A", "ABCDF12608000001"],
)
def test_ultraarm_p1_rejects_invalid_sn_code_format(sn_code):
    arm = UltraArmP1()

    with pytest.raises(ultraArmP1DataException):
        arm.calibration_parameters(
            class_name=arm.__class__.__name__,
            sn_code=sn_code,
        )


def test_ultraarm_p1_zero_calibration_delays_status_queries():
    arm = UltraArmP1()
    response_args = {}

    def response(**kwargs):
        response_args.update(kwargs)
        return "ok"

    arm._response = response

    assert arm.set_zero_calibration(0) == "ok"
    assert arm.sent_commands == ["M30 J0"]
    assert response_args == {
        "_async": True,
        "status_query_start_delay": arm.CALIBRATION_STATUS_QUERY_DELAY,
    }


def test_ultraarm_p1_update_encoder_data_sends_m92_and_returns_ok():
    arm = UltraArmP1()

    assert arm.update_encoder_data() == "ok"
    assert arm.sent_commands == ["M92"]


@pytest.mark.parametrize("joint_id", [0, 5])
def test_ultraarm_p1_set_joint_acc_rejects_invalid_joint_id(joint_id):
    arm = UltraArmP1()

    with pytest.raises(ultraArmP1DataException):
        arm.set_joint_acc(joint_id, 100)


@pytest.mark.parametrize("acc", [0, 601, "100"])
def test_ultraarm_p1_set_joint_acc_rejects_invalid_acc(acc):
    arm = UltraArmP1()

    with pytest.raises(ultraArmP1DataException):
        arm.set_joint_acc(1, acc)

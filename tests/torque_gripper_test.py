from unittest.mock import Mock

import pytest

from pymycobot.mecharm270 import MechArm270
from pymycobot.mecharmsocket import MechArmSocket
from pymycobot.mycobot280 import MyCobot280
from pymycobot.mycobot280socket import MyCobot280Socket


CLIENT_CLASSES = (
    MyCobot280,
    MyCobot280Socket,
    MechArm270,
    MechArmSocket,
)


def make_client(client_class, responses):
    client = object.__new__(client_class)
    client.get_servo_data = Mock(side_effect=responses)
    client.log = Mock()
    return client


@pytest.mark.parametrize("client_class", CLIENT_CLASSES)
@pytest.mark.parametrize(
    ("responses", "expected"),
    (
        ((3, 9), 0),
        ((3, 10), 0),
        ((3, 40), 1),
        ((4, 40), -1),
        ((None, 40), -1),
        ((3, None), -1),
        ((-1, 40), -1),
        ((3, -1), -1),
        (("3", 40), -1),
        ((3, "40"), -1),
        ((float("nan"), 40), -1),
        ((3, float("inf")), -1),
    ),
)
def test_is_torque_gripper_classifies_known_gripper_versions(
        client_class, responses, expected):
    client = make_client(client_class, responses)

    assert client.is_torque_gripper() == expected
    assert client.get_servo_data.call_args_list == [
        ((7, 0),),
        ((7, 1),),
    ]


@pytest.mark.parametrize("client_class", CLIENT_CLASSES)
def test_is_torque_gripper_logs_unknown_gripper_version(client_class):
    client = make_client(client_class, [4, 40])

    assert client.is_torque_gripper() == -1
    client.log.warning.assert_called_once_with(
        "Unknown gripper type: %s.%s", 4, 40
    )


@pytest.mark.parametrize("client_class", CLIENT_CLASSES)
def test_is_torque_gripper_returns_minus_one_on_read_error(client_class):
    client = make_client(client_class, [RuntimeError("read failed")])

    assert client.is_torque_gripper() == -1
    assert client.get_servo_data.call_args_list == [((7, 0),)]

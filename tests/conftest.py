import pytest
from unittest.mock import MagicMock, patch
from pymycobot.mycobot import MyCobot

@pytest.fixture
def mock_serial_port():
    """Mock 底层串口硬件交互，拦截对真实物理硬件的访问"""
    with patch("serial.Serial") as mock_class:
        mock_instance = MagicMock()
        mock_class.return_value = mock_instance
        
        # 模拟串口处于开启状态
        mock_instance.is_open = True
        mock_instance.isOpen.return_value = True
        
        # 默认模拟硬件串口正常响应基础协议帧
        mock_instance.read.return_value = b"\xfe\xfe\x04\x01\x01\xfa"
        
        yield mock_instance

@pytest.fixture
def mc_bot(mock_serial_port):
    """初始化并返回绑定的被测 MyCobot 实例"""
    bot = MyCobot("COM_VIRTUAL", 115200)
    return bot

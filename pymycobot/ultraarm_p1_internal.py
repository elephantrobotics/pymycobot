"""
ultraarm_p1_internal.py
Internal APIs

Author: Wang Weijian
Date: 2026-06-10
"""
from pymycobot.common import ProtocolCode


class UltraArmP1InternalMixin:

    def _check_internal_mode(self):
        """Check whether internal APIs are enabled."""
        if not getattr(self, "_internal_mode", False):
            if hasattr(self, "log"):
                self.log.error("This API is for internal use only.")
            return False
        return True

    def _internal_set_reboot(self):
        """Reboot the robot controller board.(Internal Interface)"""
        if not self._check_internal_mode():
            return -1
        with self.lock:
            self._send_command(ProtocolCode.SET_REBOOT)
            return self._response(_async=True, is_set=True)

    def _internal_get_sn_code(self):
        """For internal use only. (Prohibited from use.)"""
        if not self._check_internal_mode():
            return -1
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_SN_CODE, 'get_sn_code')

    def _internal_set_sn_code(self, sn_code):
        """For internal use only. (Prohibited from use.)"""

        if not self._check_internal_mode():
            return -1
        self.calibration_parameters(class_name=self.__class__.__name__, sn_code=sn_code)
        with self.lock:
            command = ProtocolCode.SET_SN_CODE
            command += f" {str(sn_code)}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def _internal_open_spi_log_mode(self, state):
        """Internal interface: Enable logging of SPI data forwarding"""
        if not self._check_internal_mode():
            return -1
        self.calibration_parameters(class_name=self.__class__.__name__, spi_state=state)
        with self.lock:
            command = ProtocolCode.SET_SPI_LOG_MODE
            command += f" S{str(state)}"
            self._send_command(command)
            return self._response(_async=True, is_set=True)

    def _internal_set_joint1_encoder_calibration(self):
        """Set the 730 encoder calibration for J1.(Internal Interface)"""
        if not self._check_internal_mode():
            return -1
        with self.lock:
            self._send_command(ProtocolCode.SET_J1_ENCODER_CALIBRATION_P1)
            return self._response(_async=True, is_set=True)

    def _internal_set_joint1_encoder_current_calibration(self):
        if not self._check_internal_mode():
            return -1
        """Configure the 730 encoder current calibration for joint 1 for internal use only."""
        with self.lock:
            self._send_command(ProtocolCode.SET_J1_ENCODER_CURRENT_CALIBRATION_P1)
            return self._response(_async=True, is_set=True)

    def _internal_get_joint1_encoder_calibration_state(self):
        """Read the MA730 encoder calibration status of joint 1.(Internal Interface)"""
        if not self._check_internal_mode():
            return -1
        with self.lock:
            return self._request_with_retry(ProtocolCode.GET_J1_ENCODER_CALIBRATION_STATUS, "get_encoder_calibration_state")

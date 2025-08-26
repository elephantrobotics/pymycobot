

import sys
import logging
import time

from pymycobot.log import setup_logging
from pymycobot.error import calibration_parameters
from pymycobot.common import ProtocolCode, DataProcessor


class PublicCommandGenerator(DataProcessor):
    # def __init__(self, debug=False):
    #     """
    #     Args:
    #         debug    : whether show debug info
    #     """
    #     self._version = sys.version_info[:2][0]
    #     self.debug = debug
    #     setup_logging(self.debug)
    #     self.log = logging.getLogger(__name__)
    #     self.calibration_parameters = calibration_parameters

    # def _mesg(self, genre, *args, **kwargs):
    #     """
    #     Args:
    #         genre: command type (Command)
    #         *args: other data.
    #                It is converted to octal by default.
    #                If the data needs to be encapsulated into hexadecimal,
    #                the array is used to include them. (Data cannot be nested)
    #         **kwargs: support `has_reply`
    #             has_reply: Whether there is a return value to accept.
    #     """
    #     command_data = self._process_data_command(genre, args)

    #     if genre == 178:
    #         command_data = self._encode_int16(command_data)
            
    #     elif genre in [76, 77]:
    #         command_data = [command_data[0]] + self._encode_int16(command_data[1]*10)
    #     elif genre == 115:
    #         command_data = [command_data[1],command_data[3]]

    #     LEN = len(command_data) + 2
    #     command = [
    #         ProtocolCode.HEADER,
    #         ProtocolCode.HEADER,
    #         LEN,
    #         genre,
    #         command_data,
    #         ProtocolCode.FOOTER,
    #     ]

    #     real_command = self._flatten(command)
    #     has_reply = kwargs.get("has_reply", False)
    #     return real_command, has_reply
    pass

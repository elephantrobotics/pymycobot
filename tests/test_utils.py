import os
import sys

# Add relevant ranger module to PATH... there surely is a better way to do this...
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from pymycobot import utils

port = utils.get_port_list()
print(port)

detect_result = utils.detect_port_of_basic()
print(detect_result)

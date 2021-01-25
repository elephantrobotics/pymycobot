import sys
sys.path.append('.')
import enum


class Angle(enum.Enum):

    J1 = 1
    J2 = 2
    J3 = 3
    J4 = 4
    J5 = 5
    J6 = 6


class Coord(enum.Enum):

    X  = 1
    Y  = 2
    Z  = 3
    Rx = 4
    Ry = 5
    Rz = 6

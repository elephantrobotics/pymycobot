import sys
import os
sys.path.append(os.path.dirname(__file__))
from port_setup import setup

if __name__ == "__main__":
    mc = setup()
    _sp = input('Please input speed(0-100):')
    try:
        sp = int(_sp)
    except Exception:
        print('Error: invalid speed, speed is default: 80')
        sp = 80

    while not False:
        angles_str = input('Please input angles, like ("0,0,0,0,0,0"):')
        try:
            angles_str_l = angles_str.split(',')
            if len(angles_str_l) != 6:
                raise Exception('')
            angles = [float(i) for i in angles_str_l]
        except Exception:
            print('Error: invalid angles string.')
            continue

        mc.send_angles(angles, sp)


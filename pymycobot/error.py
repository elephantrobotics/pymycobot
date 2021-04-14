import functools
from pymycobot.common import Command


class MyCobotDataException(Exception):
    pass


def check_parameters(genre=0):
    def check_decorator(func):
        @functools.wraps(func)
        def _wapper(*args, **kwargs):
            if genre == Command.SEND_ANGLE:
                check_id(args[1])
                check_angle(args[2])
                check_speed(args[3])
            elif genre == Command.SEND_ANGLES:
                check_angles(args[1])
                check_speed(args[2])
            elif genre == "radians":
                pass
            elif genre == Command.SEND_COORD:
                check_id(args[1])
                check_coord(args[1] - 1, args[2])
                check_speed(args[3])
            elif genre == Command.SEND_COORDS:
                check_coords(args[1])
                check_speed(args[2])
            elif genre == Command.SET_SPEED:
                check_speed(args[1])
            elif genre == Command.IS_IN_POSITION:
                check_boolean(args[2])
                if args[2] == 0:
                    check_angles(args[1])
                elif args[2] == 1:
                    check_coords(args[1])
            elif genre == Command.SET_GRIPPER_STATE:
                check_boolean(args[1])
                check_speed(args[2])
            elif genre == Command.SET_COLOR:
                check_rgb(args[1:])
            elif genre == Command.JOG_ANGLE or genre == Command.JOG_COORD:
                check_id(args[1])
                check_boolean(args[2])
                check_speed(args[3])
            elif genre in [
                Command.GET_JOINT_MIN_ANGLE,
                Command.GET_JOINT_MAX_ANGLE,
                Command.IS_SERVO_ENABLE,
                Command.RELEASE_SERVO,
                Command.FOCUS_SERVO,
            ]:
                check_id(args[1])

            return func(*args, **kwargs)

        return _wapper

    return check_decorator


def check_boolean(b):
    if b != 0 and b != 1:
        raise MyCobotDataException("This parameter needs to be 0 or 1")


def check_range(v, ra):
    min, max = ra
    if min <= v <= max:
        return True
    else:
        return False


def check_len(d, l, n):
    if len(d) != l:
        raise MyCobotDataException("The length of {} needs be {}".format(n, l))


def check_speed(sp):
    if not check_range(sp, [0, 100]):
        raise MyCobotDataException("speed not right, should be 0 ~ 100")


def check_id(id):
    if not check_range(id, [1, 6]):
        raise MyCobotDataException("id not right, should be 1 ~ 6")


def check_angle(v):
    if not check_range(v, [-190, 190]):
        raise MyCobotDataException("angle value not right, should be -180 ~ 180")


def check_angles(vs):
    check_len(vs, 6, "angles")
    for v in vs:
        check_angle(v)


def check_coord(id, v):
    coords = ["x", "y", "z", "rx", "ry", "rz"]
    if id < 3:
        pass
    else:
        if not check_range(v, [-180, 180]):
            raise MyCobotDataException(
                "{} value not right, should be -180 ~ 180".format(coords[id])
            )


def check_coords(vs):
    check_len(vs, 6, "coords")
    for i, v in enumerate(vs):
        check_coord(i, v)


def check_rgb(args):
    for i in args:
        if not check_range(i, [0, 255]):
            raise MyCobotDataException("The RGB value needs be 0 ~ 255")

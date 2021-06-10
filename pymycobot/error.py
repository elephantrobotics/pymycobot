_MINANGLE = -190.0
_MAXANGLE = 190.0


class MyCobotDataException(Exception):
    pass


def check_boolean(b):
    if b != 0 and b != 1:
        raise MyCobotDataException("This parameter needs to be 0 or 1")


def check_range(v, ra):
    min, max = ra
    if min <= v <= max:
        return True
    else:
        return False


def check_coord(id, v):
    coords = ["x", "y", "z", "rx", "ry", "rz"]
    if id < 3:
        pass
    else:
        if not check_range(v, [-180, 180]):
            raise MyCobotDataException(
                "{} value not right, should be -180 ~ 180".format(coords[id])
            )


def check_rgb(args):
    rgb_str = ["r", "g", "b"]
    for i, v in enumerate(args):
        if not check_range(v, [0, 255]):
            raise MyCobotDataException(
                "The RGB value needs be 0 ~ 255, but the %s is %s" % (rgb_str[i], v)
            )


def check_datas(**kwargs):
    if kwargs.get("joint_id", None) is not None and not 0 < kwargs["joint_id"] < 7:
        raise MyCobotDataException(
            "The joint id not right, should be 1 ~ 6, but received %s."
            % kwargs["joint_id"]
        )
    if (
        kwargs.get("degree", None) is not None
        and not _MINANGLE <= kwargs["degree"] <= _MAXANGLE
    ):
        raise MyCobotDataException(
            "degree value not right, should be -180 ~ 180, but received %s"
            % kwargs["degree"]
        )
    if kwargs.get("len6", None) is not None:
        len_ = len(kwargs["len6"])
        if len_ != 6:
            raise MyCobotDataException(
                "The length of data should be 6, the truth is %s" % len_
            )
    if kwargs.get("degrees", None) is not None:
        for idx, angle in enumerate(kwargs["degrees"]):
            if not _MINANGLE <= angle <= _MAXANGLE:
                raise MyCobotDataException(
                    "degree value not right, should be -180 ~ 180, the error index is %s"
                    % idx
                )
    if kwargs.get("speed", None) is not None and not 0 <= kwargs["speed"] <= 100:
        raise MyCobotDataException(
            "speed value not right, should be 0 ~ 100, the error speed is %s"
            % kwargs["speed"]
        )
    if kwargs.get("rgb", None) is not None:
        check_rgb(kwargs["rgb"])

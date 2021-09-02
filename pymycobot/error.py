# coding=utf-8

# In order to adapt to the atom side, ID of 0-5 or 1-6 are allowed.
# In order to support end control, ID 7 is allowed.
MIN_ID = 0
MAX_ID = 7

# In fact, most joints cannot reach plus or minus 180 degrees.
# There may be a value greater than 180 when reading the angle,
# and the maximum and minimum values are expanded for compatibility.
MIN_ANGLE = -190.0
MAX_ANGLE = 190.0


class MyCobotDataException(Exception):
    pass


def check_boolean(b):
    if b != 0 and b != 1:
        raise MyCobotDataException("This parameter needs to be 0 or 1")


def calibration_parameters(**kwargs):
    if kwargs.get("id", None) is not None and not MIN_ID <= kwargs["id"] <= MAX_ID:
        raise MyCobotDataException(
            "The id not right, should be {0} ~ {1}, but received {2}.".format(
                MIN_ID, MAX_ID, kwargs["id"]
            )
        )

    if (
        kwargs.get("degree", None) is not None
        and not MIN_ANGLE <= kwargs["degree"] <= MAX_ANGLE
    ):
        raise MyCobotDataException(
            "degree value not right, should be {0} ~ {1}, but received {2}".format(
                MIN_ANGLE, MAX_ANGLE, kwargs["degree"]
            )
        )

    if kwargs.get("degrees", None) is not None:
        degrees = kwargs["degrees"]
        if not isinstance(degrees, list):
            raise MyCobotDataException("`degrees` must be a list.")
        if len(degrees) != 6:
            raise MyCobotDataException("The length of `degrees` must be 6.")
        for idx, angle in enumerate(degrees):
            if not MIN_ANGLE <= angle <= MAX_ANGLE:
                raise MyCobotDataException(
                    "Has invalid degree value, error on index {0}. Degree should be {1} ~ {2}.".format(
                        idx, MIN_ANGLE, MAX_ANGLE
                    )
                )

    if kwargs.get("coords", None) is not None:
        coords = kwargs["coords"]
        if not isinstance(coords, list):
            raise MyCobotDataException("`coords` must be a list.")
        if len(coords) != 6:
            raise MyCobotDataException("The length of `coords` must be 6.")

    if kwargs.get("speed", None) is not None and not 0 <= kwargs["speed"] <= 100:
        raise MyCobotDataException(
            "speed value not right, should be 0 ~ 100, the error speed is %s"
            % kwargs["speed"]
        )

    if kwargs.get("rgb", None) is not None:
        rgb_str = ["r", "g", "b"]
        for i, v in enumerate(kwargs["rgb"]):
            if not (0 <= v <= 255):
                raise MyCobotDataException(
                    "The RGB value needs be 0 ~ 255, but the %s is %s" % (rgb_str[i], v)
                )

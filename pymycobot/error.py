import functools


class MyCobotDataException(Exception):
    pass

def check_id(func):

    @functools.wraps(func)
    def _wapper(*args, **kwargs):
        if not 1 <= args[1] <= 6:
            raise MyCobotDataException('id not right, should be 1 ~ 6')

        return func(*args, **kwargs)

    return _wapper



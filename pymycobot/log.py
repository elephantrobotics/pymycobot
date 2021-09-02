# coding=utf-8

import logging


def setup_logging(debug=False):
    root_logger = logging.getLogger()

    debug_fomatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(levelname).4s [%(name)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    logger_handle = logging.StreamHandler()
    logger_handle.setFormatter(debug_fomatter)
    if debug:
        logger_handle.setLevel(logging.DEBUG)
    else:
        logger_handle.setLevel(logging.WARNING)

    root_logger.addHandler(logger_handle)
    root_logger.setLevel(0)

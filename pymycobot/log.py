# coding=utf-8

import logging
import logging.handlers


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
        # 100M日志
        save = logging.handlers.RotatingFileHandler(
        "python_debug.log", maxBytes=100*1024*1024, backupCount=1)
        save.setFormatter(debug_fomatter)
        root_logger.addHandler(save)
    else:
        logger_handle.setLevel(logging.WARNING)

    root_logger.addHandler(logger_handle)
    root_logger.setLevel(0)

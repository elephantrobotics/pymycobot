# coding=utf-8

import logging
import logging.handlers
from datetime import datetime


def setup_logging(debug=False):
    # logging.basicConfig()
    root_logger = logging.getLogger()

    if root_logger.handlers:
        return root_logger

    debug_fomatter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(levelname).4s [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger_handle = logging.StreamHandler()
    logger_handle.setFormatter(debug_fomatter)
    if debug:
        logger_handle.setLevel(logging.DEBUG)

        log_name = datetime.now().strftime("python_debug_%Y%m%d.log")

        # 100M日志
        save = logging.handlers.RotatingFileHandler(
        log_name, maxBytes=100*1024*1024, backupCount=3)
        save.setFormatter(debug_fomatter)
        root_logger.addHandler(save)
        root_logger.setLevel(logging.DEBUG)
    else:
        logger_handle.setLevel(logging.WARNING)
        root_logger.setLevel(logging.WARNING)

    root_logger.addHandler(logger_handle)
    # root_logger.setLevel(0)

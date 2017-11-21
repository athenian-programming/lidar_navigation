import argparse
import logging

from constants import HTTP_DELAY_SECS, HTTP_FILE, LOG_LEVEL
from constants import HTTP_DELAY_SECS_DEFAULT, HTTP_HOST_DEFAULT, HTTP_TEMPLATE_DEFAULT
from constants import HTTP_HOST


def setup_cli_args(*args):
    parser = argparse.ArgumentParser()
    for arg in args:
        if type(arg) is list:
            for a in arg:
                a(parser)
        else:
            arg(parser)
    return vars(parser.parse_args())


def http_host(p):
    return p.add_argument("--http", dest=HTTP_HOST, default=HTTP_HOST_DEFAULT,
                          help="HTTP hostname:port [{0}]".format(HTTP_HOST_DEFAULT))


def http_delay_secs(p):
    return p.add_argument("--delay", "--http_delay", dest=HTTP_DELAY_SECS, default=HTTP_DELAY_SECS_DEFAULT, type=float,
                          help="HTTP delay secs [{0}]".format(HTTP_DELAY_SECS_DEFAULT))


def http_file(p):
    return p.add_argument("-i", "--file", "--http_file", dest=HTTP_FILE, default=HTTP_TEMPLATE_DEFAULT,
                          help="HTTP template file")


def http_verbose(p):
    return p.add_argument("--http_verbose", "--verbose_http", dest="http_verbose", default=False, action="store_true",
                          help="Enable verbose HTTP log [false]")


def log_level(p):
    return p.add_argument("-v", "--verbose", dest=LOG_LEVEL, default=logging.INFO, action="store_const",
                          const=logging.DEBUG, help="Enable debugging info")

import os

import sys

__path = os.path.abspath(sys.modules[__name__].__file__)
__dirname = os.path.dirname(__path)
HTTP_TEMPLATE_DEFAULT = __dirname + "/html/plot-image.html"
HTTP_PORT_DEFAULT = 8080
HTTP_HOST_DEFAULT = "localhost:{0}".format(HTTP_PORT_DEFAULT)
HTTP_DELAY_SECS_DEFAULT = 0

HTTP_HOST = "http_host"
HTTP_FILE = "http_file"
HTTP_DELAY_SECS = "http_delay_secs"
HTTP_VERBOSE = "http_verbose"
LOG_LEVEL = "loglevel"

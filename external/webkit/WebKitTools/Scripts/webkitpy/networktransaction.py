import time

from mechanize import HTTPError
from webkitpy.webkit_logging import log


class NetworkTimeout(Exception):
    pass


class NetworkTransaction(object):
    def __init__(self, initial_backoff_seconds=10, grown_factor=1.1, timeout_seconds=5*60*60):
        self._initial_backoff_seconds = initial_backoff_seconds
        self._grown_factor = grown_factor
        self._timeout_seconds = timeout_seconds

    def run(self, request):
        self._total_sleep = 0
        self._backoff_seconds = self._initial_backoff_seconds
        while True:
            try:
                return request()
            except HTTPError, e:
                self._check_for_timeout()
                log("Received HTTP status %s from server.  Retrying in %s seconds..." % (e.code, self._backoff_seconds))
                self._sleep()

    def _check_for_timeout(self):
        if self._total_sleep + self._backoff_seconds > self._timeout_seconds:
            raise NetworkTimeout()

    def _sleep(self):
        time.sleep(self._backoff_seconds)
        self._total_sleep += self._backoff_seconds
        self._backoff_seconds *= self._grown_factor

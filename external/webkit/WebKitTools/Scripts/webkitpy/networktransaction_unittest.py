import unittest

from mechanize import HTTPError
from webkitpy.networktransaction import NetworkTransaction, NetworkTimeout

class NetworkTransactionTest(unittest.TestCase):
    exception = Exception("Test exception")

    def test_success(self):
        transaction = NetworkTransaction()
        self.assertEqual(transaction.run(lambda: 42), 42)

    def _raise_exception(self):
        raise self.exception

    def test_exception(self):
        transaction = NetworkTransaction()
        did_process_exception = False
        did_throw_exception = True
        try:
            transaction.run(lambda: self._raise_exception())
            did_throw_exception = False
        except Exception, e:
            did_process_exception = True
            self.assertEqual(e, self.exception)
        self.assertTrue(did_throw_exception)
        self.assertTrue(did_process_exception)

    def _raise_http_error(self):
        self._run_count += 1
        if self._run_count < 3:
            raise HTTPError("http://example.com/", 500, "inteneral server error", None, None)
        return 42

    def test_retry(self):
        self._run_count = 0
        transaction = NetworkTransaction(initial_backoff_seconds=0)
        self.assertEqual(transaction.run(lambda: self._raise_http_error()), 42)
        self.assertEqual(self._run_count, 3)

    def test_timeout(self):
        self._run_count = 0
        transaction = NetworkTransaction(initial_backoff_seconds=60*60, timeout_seconds=60)
        did_process_exception = False
        did_throw_exception = True
        try:
            transaction.run(lambda: self._raise_http_error())
            did_throw_exception = False
        except NetworkTimeout, e:
            did_process_exception = True
        self.assertTrue(did_throw_exception)
        self.assertTrue(did_process_exception)

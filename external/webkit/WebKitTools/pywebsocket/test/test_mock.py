#!/usr/bin/env python
"""Tests for mock module."""


import Queue
import threading
import unittest

import mock


class MockConnTest(unittest.TestCase):
    def setUp(self):
        self._conn = mock.MockConn('ABC\r\nDEFG\r\n\r\nHIJK')

    def test_readline(self):
        self.assertEqual('ABC\r\n', self._conn.readline())
        self.assertEqual('DEFG\r\n', self._conn.readline())
        self.assertEqual('\r\n', self._conn.readline())
        self.assertEqual('HIJK', self._conn.readline())
        self.assertEqual('', self._conn.readline())

    def test_read(self):
        self.assertEqual('ABC\r\nD', self._conn.read(6))
        self.assertEqual('EFG\r\n\r\nHI', self._conn.read(9))
        self.assertEqual('JK', self._conn.read(10))
        self.assertEqual('', self._conn.read(10))

    def test_read_and_readline(self):
        self.assertEqual('ABC\r\nD', self._conn.read(6))
        self.assertEqual('EFG\r\n', self._conn.readline())
        self.assertEqual('\r\nHIJK', self._conn.read(9))
        self.assertEqual('', self._conn.readline())

    def test_write(self):
        self._conn.write('Hello\r\n')
        self._conn.write('World\r\n')
        self.assertEqual('Hello\r\nWorld\r\n', self._conn.written_data())


class MockBlockingConnTest(unittest.TestCase):
    def test_read(self):
        class LineReader(threading.Thread):
            def __init__(self, conn, queue):
                threading.Thread.__init__(self)
                self._queue = queue
                self._conn = conn
                self.setDaemon(True)
                self.start()
            def run(self):
                while True:
                    data = self._conn.readline()
                    self._queue.put(data)
        conn = mock.MockBlockingConn()
        queue = Queue.Queue()
        reader = LineReader(conn, queue)
        self.failUnless(queue.empty())
        conn.put_bytes('Foo bar\r\n')
        read = queue.get()
        self.assertEqual('Foo bar\r\n', read)


class MockTableTest(unittest.TestCase):
    def test_create_from_dict(self):
        table = mock.MockTable({'Key':'Value'})
        self.assertEqual('Value', table.get('KEY'))
        self.assertEqual('Value', table['key'])

    def test_create_from_list(self):
        table = mock.MockTable([('Key', 'Value')])
        self.assertEqual('Value', table.get('KEY'))
        self.assertEqual('Value', table['key'])

    def test_create_from_tuple(self):
        table = mock.MockTable((('Key', 'Value'),))
        self.assertEqual('Value', table.get('KEY'))
        self.assertEqual('Value', table['key'])

    def test_set_and_get(self):
        table = mock.MockTable()
        self.assertEqual(None, table.get('Key'))
        table['Key'] = 'Value'
        self.assertEqual('Value', table.get('Key'))
        self.assertEqual('Value', table.get('key'))
        self.assertEqual('Value', table.get('KEY'))
        self.assertEqual('Value', table['Key'])
        self.assertEqual('Value', table['key'])
        self.assertEqual('Value', table['KEY'])


if __name__ == '__main__':
    unittest.main()


# vi:sts=4 sw=4 et

#!/usr/bin/env python
"""Tests for memorizingfile module."""


import StringIO
import unittest

import config  # This must be imported before mod_pywebsocket.
from mod_pywebsocket import memorizingfile


class UtilTest(unittest.TestCase):
    def check(self, memorizing_file, num_read, expected_list):
        for unused in range(num_read):
            memorizing_file.readline()
        actual_list = memorizing_file.get_memorized_lines()
        self.assertEqual(len(expected_list), len(actual_list))
        for expected, actual in zip(expected_list, actual_list):
            self.assertEqual(expected, actual)

    def test_get_memorized_lines(self):
        memorizing_file = memorizingfile.MemorizingFile(StringIO.StringIO(
                'Hello\nWorld\nWelcome'))
        self.check(memorizing_file, 3, ['Hello\n', 'World\n', 'Welcome'])

    def test_get_memorized_lines_limit_memorized_lines(self):
        memorizing_file = memorizingfile.MemorizingFile(StringIO.StringIO(
                'Hello\nWorld\nWelcome'), 2)
        self.check(memorizing_file, 3, ['Hello\n', 'World\n'])

    def test_get_memorized_lines_empty_file(self):
        memorizing_file = memorizingfile.MemorizingFile(StringIO.StringIO(
                ''))
        self.check(memorizing_file, 10, [])


if __name__ == '__main__':
    unittest.main()


# vi:sts=4 sw=4 et

#!/usr/bin/env python
"""Verifies that Google Test warns the user when not initialized properly."""

__author__ = 'wan@google.com (Zhanyong Wan)'

import gtest_test_utils


COMMAND = gtest_test_utils.GetTestExecutablePath('gtest_uninitialized_test_')


def Assert(condition):
  if not condition:
    raise AssertionError


def AssertEq(expected, actual):
  if expected != actual:
    print 'Expected: %s' % (expected,)
    print '  Actual: %s' % (actual,)
    raise AssertionError


def TestExitCodeAndOutput(command):
  """Runs the given command and verifies its exit code and output."""

  # Verifies that 'command' exits with code 1.
  p = gtest_test_utils.Subprocess(command)
  Assert(p.exited)
  AssertEq(1, p.exit_code)
  Assert('InitGoogleTest' in p.output)


class GTestUninitializedTest(gtest_test_utils.TestCase):
  def testExitCodeAndOutput(self):
    TestExitCodeAndOutput(COMMAND)


if __name__ == '__main__':
  gtest_test_utils.Main()

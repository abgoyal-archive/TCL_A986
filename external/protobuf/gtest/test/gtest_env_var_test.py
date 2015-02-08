#!/usr/bin/env python
"""Verifies that Google Test correctly parses environment variables."""

__author__ = 'wan@google.com (Zhanyong Wan)'

import os
import gtest_test_utils


IS_WINDOWS = os.name == 'nt'
IS_LINUX = os.name == 'posix' and os.uname()[0] == 'Linux'

COMMAND = gtest_test_utils.GetTestExecutablePath('gtest_env_var_test_')


def AssertEq(expected, actual):
  if expected != actual:
    print 'Expected: %s' % (expected,)
    print '  Actual: %s' % (actual,)
    raise AssertionError


def SetEnvVar(env_var, value):
  """Sets the env variable to 'value'; unsets it when 'value' is None."""

  if value is not None:
    os.environ[env_var] = value
  elif env_var in os.environ:
    del os.environ[env_var]


def GetFlag(flag):
  """Runs gtest_env_var_test_ and returns its output."""

  args = [COMMAND]
  if flag is not None:
    args += [flag]
  return gtest_test_utils.Subprocess(args).output


def TestFlag(flag, test_val, default_val):
  """Verifies that the given flag is affected by the corresponding env var."""

  env_var = 'GTEST_' + flag.upper()
  SetEnvVar(env_var, test_val)
  AssertEq(test_val, GetFlag(flag))
  SetEnvVar(env_var, None)
  AssertEq(default_val, GetFlag(flag))


class GTestEnvVarTest(gtest_test_utils.TestCase):
  def testEnvVarAffectsFlag(self):
    """Tests that environment variable should affect the corresponding flag."""

    TestFlag('break_on_failure', '1', '0')
    TestFlag('color', 'yes', 'auto')
    TestFlag('filter', 'FooTest.Bar', '*')
    TestFlag('output', 'xml:tmp/foo.xml', '')
    TestFlag('print_time', '0', '1')
    TestFlag('repeat', '999', '1')
    TestFlag('throw_on_failure', '1', '0')
    TestFlag('death_test_style', 'threadsafe', 'fast')

    if IS_WINDOWS:
      TestFlag('catch_exceptions', '1', '0')

    if IS_LINUX:
      TestFlag('death_test_use_fork', '1', '0')
      TestFlag('stack_trace_depth', '0', '100')


if __name__ == '__main__':
  gtest_test_utils.Main()

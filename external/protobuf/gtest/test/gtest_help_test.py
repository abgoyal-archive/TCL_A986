#!/usr/bin/env python
"""Tests the --help flag of Google C++ Testing Framework.

SYNOPSIS
       gtest_help_test.py --gtest_build_dir=BUILD/DIR
         # where BUILD/DIR contains the built gtest_help_test_ file.
       gtest_help_test.py
"""

__author__ = 'wan@google.com (Zhanyong Wan)'

import os
import re
import gtest_test_utils


IS_WINDOWS = os.name == 'nt'

PROGRAM_PATH = gtest_test_utils.GetTestExecutablePath('gtest_help_test_')
FLAG_PREFIX = '--gtest_'
CATCH_EXCEPTIONS_FLAG = FLAG_PREFIX + 'catch_exceptions'
DEATH_TEST_STYLE_FLAG = FLAG_PREFIX + 'death_test_style'

# The help message must match this regex.
HELP_REGEX = re.compile(
    FLAG_PREFIX + r'list_tests.*' +
    FLAG_PREFIX + r'filter=.*' +
    FLAG_PREFIX + r'also_run_disabled_tests.*' +
    FLAG_PREFIX + r'repeat=.*' +
    FLAG_PREFIX + r'shuffle.*' +
    FLAG_PREFIX + r'random_seed=.*' +
    FLAG_PREFIX + r'color=.*' +
    FLAG_PREFIX + r'print_time.*' +
    FLAG_PREFIX + r'output=.*' +
    FLAG_PREFIX + r'break_on_failure.*' +
    FLAG_PREFIX + r'throw_on_failure.*',
    re.DOTALL)


def RunWithFlag(flag):
  """Runs gtest_help_test_ with the given flag.

  Returns:
    the exit code and the text output as a tuple.
  Args:
    flag: the command-line flag to pass to gtest_help_test_, or None.
  """

  if flag is None:
    command = [PROGRAM_PATH]
  else:
    command = [PROGRAM_PATH, flag]
  child = gtest_test_utils.Subprocess(command)
  return child.exit_code, child.output


class GTestHelpTest(gtest_test_utils.TestCase):
  """Tests the --help flag and its equivalent forms."""

  def TestHelpFlag(self, flag):
    """Verifies that the right message is printed and the tests are
    skipped when the given flag is specified."""

    exit_code, output = RunWithFlag(flag)
    self.assertEquals(0, exit_code)
    self.assert_(HELP_REGEX.search(output), output)
    if IS_WINDOWS:
      self.assert_(CATCH_EXCEPTIONS_FLAG in output, output)
      self.assert_(DEATH_TEST_STYLE_FLAG not in output, output)
    else:
      self.assert_(CATCH_EXCEPTIONS_FLAG not in output, output)
      self.assert_(DEATH_TEST_STYLE_FLAG in output, output)

  def testPrintsHelpWithFullFlag(self):
    self.TestHelpFlag('--help')

  def testPrintsHelpWithShortFlag(self):
    self.TestHelpFlag('-h')

  def testPrintsHelpWithQuestionFlag(self):
    self.TestHelpFlag('-?')

  def testPrintsHelpWithWindowsStyleQuestionFlag(self):
    self.TestHelpFlag('/?')

  def testRunsTestsWithoutHelpFlag(self):
    """Verifies that when no help flag is specified, the tests are run
    and the help message is not printed."""

    exit_code, output = RunWithFlag(None)
    self.assert_(exit_code != 0)
    self.assert_(not HELP_REGEX.search(output), output)


if __name__ == '__main__':
  gtest_test_utils.Main()

#!/usr/bin/env python
"""Negative compilation test for Google Test."""

__author__ = 'wan@google.com (Zhanyong Wan)'

import os
import sys
import unittest


IS_LINUX = os.name == 'posix' and os.uname()[0] == 'Linux'
if not IS_LINUX:
  sys.exit(0)  # Negative compilation tests are not supported on Windows & Mac.


class GTestNCTest(unittest.TestCase):
  """Negative compilation test for Google Test."""

  def testCompilerError(self):
    """Verifies that erroneous code leads to expected compiler
    messages."""

    # Defines a list of test specs, where each element is a tuple
    # (test name, list of regexes for matching the compiler errors).
    test_specs = [
      ('CANNOT_IGNORE_RUN_ALL_TESTS_RESULT',
       [r'ignoring return value']),

      ('USER_CANNOT_INCLUDE_GTEST_INTERNAL_INL_H',
       [r'must not be included except by Google Test itself']),

      ('CATCHES_DECLARING_SETUP_IN_TEST_FIXTURE_WITH_TYPO',
       [r'Setup_should_be_spelled_SetUp']),

      ('CATCHES_CALLING_SETUP_IN_TEST_WITH_TYPO',
       [r'Setup_should_be_spelled_SetUp']),

      ('CATCHES_DECLARING_SETUP_IN_ENVIRONMENT_WITH_TYPO',
       [r'Setup_should_be_spelled_SetUp']),

      ('CATCHES_CALLING_SETUP_IN_ENVIRONMENT_WITH_TYPO',
       [r'Setup_should_be_spelled_SetUp']),

      ('CATCHES_WRONG_CASE_IN_TYPED_TEST_P',
       [r'BarTest.*was not declared']),

      ('CATCHES_WRONG_CASE_IN_REGISTER_TYPED_TEST_CASE_P',
       [r'BarTest.*was not declared']),

      ('CATCHES_WRONG_CASE_IN_INSTANTIATE_TYPED_TEST_CASE_P',
       [r'BarTest.*not declared']),

      ('CATCHES_INSTANTIATE_TYPED_TESET_CASE_P_WITH_SAME_NAME_PREFIX',
       [r'redefinition of.*My.*FooTest']),

      ('STATIC_ASSERT_TYPE_EQ_IS_NOT_A_TYPE',
       [r'StaticAssertTypeEq.* does not name a type']),

      ('STATIC_ASSERT_TYPE_EQ_WORKS_IN_NAMESPACE',
       [r'StaticAssertTypeEq.*int.*const int']),

      ('STATIC_ASSERT_TYPE_EQ_WORKS_IN_CLASS',
       [r'StaticAssertTypeEq.*int.*bool']),

      ('STATIC_ASSERT_TYPE_EQ_WORKS_IN_FUNCTION',
       [r'StaticAssertTypeEq.*const int.*int']),

      ('SANITY',
       None)
      ]

    # TODO(wan@google.com): verify that the test specs are satisfied.


if __name__ == '__main__':
  unittest.main()

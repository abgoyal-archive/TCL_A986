#!/usr/bin/env python
"""Run all tests in the same directory.
"""


import os
import re
import unittest


_TEST_MODULE_PATTERN = re.compile(r'^(test_.+)\.py$')


def _list_test_modules(directory):
    module_names = []
    for filename in os.listdir(directory):
        match = _TEST_MODULE_PATTERN.search(filename)
        if match:
            module_names.append(match.group(1))
    return module_names


def _suite():
    loader = unittest.TestLoader()
    return loader.loadTestsFromNames(
            _list_test_modules(os.path.join(os.path.split(__file__)[0], '.')))


if __name__ == '__main__':
    unittest.main(defaultTest='_suite')


# vi:sts=4 sw=4 et

#!/usr/bin/python
"""Unit test for text_style.py."""

import unittest

import text as text_style
from text import TextProcessor

class TextStyleTestCase(unittest.TestCase):
    """TestCase for text_style.py"""

    def assertNoError(self, lines):
        """Asserts that the specified lines has no errors."""
        self.had_error = False

        def error_for_test(line_number, category, confidence, message):
            """Records if an error occurs."""
            self.had_error = True

        text_style.process_file_data('', lines, error_for_test)
        self.assert_(not self.had_error, '%s should not have any errors.' % lines)

    def assertError(self, lines, expected_line_number):
        """Asserts that the specified lines has an error."""
        self.had_error = False

        def error_for_test(line_number, category, confidence, message):
            """Checks if the expected error occurs."""
            self.assertEquals(expected_line_number, line_number)
            self.assertEquals('whitespace/tab', category)
            self.had_error = True

        text_style.process_file_data('', lines, error_for_test)
        self.assert_(self.had_error, '%s should have an error [whitespace/tab].' % lines)


    def test_no_error(self):
        """Tests for no error cases."""
        self.assertNoError([''])
        self.assertNoError(['abc def', 'ggg'])


    def test_error(self):
        """Tests for error cases."""
        self.assertError(['2009-12-16\tKent Tamura\t<tkent@chromium.org>'], 1)
        self.assertError(['2009-12-16 Kent Tamura <tkent@chromium.org>',
                          '',
                          '\tReviewed by NOBODY.'], 3)


class TextProcessorTest(unittest.TestCase):

    """Tests TextProcessor class."""

    def mock_handle_style_error(self):
        pass

    def test_init(self):
        """Test __init__ constructor."""
        processor = TextProcessor("foo.txt", self.mock_handle_style_error)
        self.assertEquals(processor.file_path, "foo.txt")
        self.assertEquals(processor.handle_style_error, self.mock_handle_style_error)


if __name__ == '__main__':
    unittest.main()

"""Checks WebKit style for text files."""


class TextProcessor(object):

    """Processes text lines for checking style."""

    def __init__(self, file_path, handle_style_error):
        self.file_path = file_path
        self.handle_style_error = handle_style_error

    def process(self, lines):
        lines = (["// adjust line numbers to make the first line 1."] + lines)

        # FIXME: share with cpp_style.
        for line_number, line in enumerate(lines):
            if "\t" in line:
                self.handle_style_error(line_number,
                                        "whitespace/tab", 5,
                                        "Line contains tab character.")


# FIXME: Remove this function (requires refactoring unit tests).
def process_file_data(filename, lines, error):
    processor = TextProcessor(filename, error)
    processor.process(lines)


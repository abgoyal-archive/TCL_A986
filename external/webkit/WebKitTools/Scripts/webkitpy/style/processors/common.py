"""Supports style checking not specific to any one processor."""


# FIXME: Test this list in the same way that the list of CppProcessor
#        categories is tested, for example by checking that all of its
#        elements appear in the unit tests. This should probably be done
#        after moving the relevant cpp_unittest.ErrorCollector code
#        into a shared location and refactoring appropriately.
categories = set([
    "whitespace/carriage_return",
])


def check_no_carriage_return(line, line_number, error):
    """Check that a line does not end with a carriage return.

    Returns true if the check is successful (i.e. if the line does not
    end with a carriage return), and false otherwise.

    Args:
      line: A string that is the line to check.
      line_number: The line number.
      error: The function to call with any errors found.

    """

    if line.endswith("\r"):
        error(line_number,
              "whitespace/carriage_return",
              1,
              "One or more unexpected \\r (^M) found; "
              "better to use only a \\n")
        return False

    return True



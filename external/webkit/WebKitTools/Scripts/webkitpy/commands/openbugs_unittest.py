from webkitpy.commands.commandtest import CommandsTest
from webkitpy.commands.openbugs import OpenBugs

class OpenBugsTest(CommandsTest):

    find_bugs_in_string_expectations = [
        ["123", []],
        ["1234", ["1234"]],
        ["12345", ["12345"]],
        ["123456", ["123456"]],
        ["1234567", []],
        [" 123456 234567", ["123456", "234567"]],
    ]

    def test_find_bugs_in_string(self):
        openbugs = OpenBugs()
        for expectation in self.find_bugs_in_string_expectations:
            self.assertEquals(openbugs._find_bugs_in_string(expectation[0]), expectation[1])

    def test_args_parsing(self):
        expected_stderr = "2 bugs found in input.\nMOCK: user.open_url: http://example.com/12345\nMOCK: user.open_url: http://example.com/23456\n"
        self.assert_execute_outputs(OpenBugs(), ["12345\n23456"], expected_stderr=expected_stderr)

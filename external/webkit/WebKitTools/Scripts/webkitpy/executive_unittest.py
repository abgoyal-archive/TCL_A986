import unittest
from webkitpy.executive import Executive, run_command

class ExecutiveTest(unittest.TestCase):

    def test_run_command_with_bad_command(self):
        def run_bad_command():
            run_command(["foo_bar_command_blah"], error_handler=Executive.ignore_error, return_exit_code=True)
        self.failUnlessRaises(OSError, run_bad_command)

if __name__ == '__main__':
    unittest.main()

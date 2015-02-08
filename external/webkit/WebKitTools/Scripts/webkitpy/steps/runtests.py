from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log

class RunTests(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.build,
            Options.test,
            Options.non_interactive,
            Options.quiet,
            Options.port,
        ]

    def run(self, state):
        if not self._options.build:
            return
        if not self._options.test:
            return

        # Run the scripting unit tests first because they're quickest.
        log("Running Python unit tests")
        self._tool.executive.run_and_throw_if_fail(self.port().run_python_unittests_command())
        log("Running Perl unit tests")
        self._tool.executive.run_and_throw_if_fail(self.port().run_perl_unittests_command())
        log("Running JavaScriptCore tests")
        self._tool.executive.run_and_throw_if_fail(self.port().run_javascriptcore_tests_command(), quiet=True)

        log("Running run-webkit-tests")
        args = self.port().run_webkit_tests_command()
        if self._options.non_interactive:
            args.append("--no-launch-safari")
            args.append("--exit-after-n-failures=1")
        if self._options.quiet:
            args.append("--quiet")
        self._tool.executive.run_and_throw_if_fail(args)


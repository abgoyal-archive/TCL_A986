from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import log


class Build(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.build,
            Options.quiet,
            Options.build_style,
        ]

    def build(self, build_style):
        self._tool.executive.run_and_throw_if_fail(self.port().build_webkit_command(build_style=build_style), self._options.quiet)

    def run(self, state):
        if not self._options.build:
            return
        log("Building WebKit")
        if self._options.build_style == "both":
            self.build("debug")
            self.build("release")
        else:
            self.build(self._options.build_style)

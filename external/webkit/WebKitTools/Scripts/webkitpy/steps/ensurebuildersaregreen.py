from webkitpy.steps.abstractstep import AbstractStep
from webkitpy.steps.options import Options
from webkitpy.webkit_logging import error


class EnsureBuildersAreGreen(AbstractStep):
    @classmethod
    def options(cls):
        return [
            Options.check_builders,
        ]

    def run(self, state):
        if not self._options.check_builders:
            return
        red_builders_names = self._tool.buildbot.red_core_builders_names()
        if not red_builders_names:
            return
        red_builders_names = map(lambda name: "\"%s\"" % name, red_builders_names) # Add quotes around the names.
        error("Builders [%s] are red, please do not commit.\nSee http://%s.\nPass --ignore-builders to bypass this check." % (", ".join(red_builders_names), self._tool.buildbot.buildbot_host))

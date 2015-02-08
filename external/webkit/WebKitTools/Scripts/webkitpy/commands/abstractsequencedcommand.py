from webkitpy.multicommandtool import AbstractDeclarativeCommand
from webkitpy.stepsequence import StepSequence


class AbstractSequencedCommand(AbstractDeclarativeCommand):
    steps = None
    def __init__(self):
        self._sequence = StepSequence(self.steps)
        AbstractDeclarativeCommand.__init__(self, self._sequence.options())

    def _prepare_state(self, options, args, tool):
        return None

    def execute(self, options, args, tool):
        self._sequence.run_and_handle_errors(tool, options, self._prepare_state(options, args, tool))

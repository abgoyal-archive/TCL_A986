from webkitpy.steps.abstractstep import AbstractStep


# FIXME: Unify with StepSequence?  I'm not sure yet which is the better design.
class MetaStep(AbstractStep):
    substeps = [] # Override in subclasses
    def __init__(self, tool, options):
        AbstractStep.__init__(self, tool, options)
        self._step_instances = []
        for step_class in self.substeps:
            self._step_instances.append(step_class(tool, options))

    @staticmethod
    def _collect_options_from_steps(steps):
        collected_options = []
        for step in steps:
            collected_options = collected_options + step.options()
        return collected_options

    @classmethod
    def options(cls):
        return cls._collect_options_from_steps(cls.substeps)

    def run(self, state):
        for step in self._step_instances:
             step.run(state)

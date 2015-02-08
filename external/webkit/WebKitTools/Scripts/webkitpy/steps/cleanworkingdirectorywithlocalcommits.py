from webkitpy.steps.cleanworkingdirectory import CleanWorkingDirectory

class CleanWorkingDirectoryWithLocalCommits(CleanWorkingDirectory):
    def __init__(self, tool, options):
        # FIXME: This a bit of a hack.  Consider doing this more cleanly.
        CleanWorkingDirectory.__init__(self, tool, options, allow_local_commits=True)

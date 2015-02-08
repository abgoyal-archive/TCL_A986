from webkitpy.webkit_logging import log
from webkitpy.webkitport import WebKitPort


class AbstractStep(object):
    def __init__(self, tool, options):
        self._tool = tool
        self._options = options
        self._port = None

    def _run_script(self, script_name, quiet=False, port=WebKitPort):
        log("Running %s" % script_name)
        # FIXME: This should use self.port()
        self._tool.executive.run_and_throw_if_fail(port.script_path(script_name), quiet)

    # FIXME: The port should live on the tool.
    def port(self):
        if self._port:
            return self._port
        self._port = WebKitPort.port(self._options.port)
        return self._port

    _well_known_keys = {
        "diff" : lambda self: self._tool.scm().create_patch(),
        "changelogs" : lambda self: self._tool.scm().modified_changelogs(),
    }

    def cached_lookup(self, state, key, promise=None):
        if state.get(key):
            return state[key]
        if not promise:
            promise = self._well_known_keys.get(key)
        state[key] = promise(self)
        return state[key]

    @classmethod
    def options(cls):
        return []

    def run(self, state):
        raise NotImplementedError, "subclasses must implement"

import os

from optparse import make_option
from webkitpy.executive import Executive


class WebKitPort(object):

    # We might need to pass scm into this function for scm.checkout_root
    @classmethod
    def script_path(cls, script_name):
        return os.path.join("WebKitTools", "Scripts", script_name)

    @staticmethod
    def port(port_name):
        ports = {
            "chromium": ChromiumPort,
            "gtk": GtkPort,
            "mac": MacPort,
            "qt": QtPort,
        }
        # FIXME: We should default to WinPort on Windows.
        return ports.get(port_name, MacPort)

    @classmethod
    def name(cls):
        raise NotImplementedError("subclasses must implement")

    @classmethod
    def flag(cls):
        raise NotImplementedError("subclasses must implement")

    @classmethod
    def update_webkit_command(cls):
        return [cls.script_path("update-webkit")]

    @classmethod
    def build_webkit_command(cls, build_style=None):
        command = [cls.script_path("build-webkit")]
        if build_style == "debug":
            command.append("--debug")
        if build_style == "release":
            command.append("--release")
        return command

    @classmethod
    def run_javascriptcore_tests_command(cls):
        return [cls.script_path("run-javascriptcore-tests")]

    @classmethod
    def run_webkit_tests_command(cls):
        return [cls.script_path("run-webkit-tests")]

    @classmethod
    def run_python_unittests_command(cls):
        return [cls.script_path("test-webkitpy")]

    @classmethod
    def run_perl_unittests_command(cls):
        return [cls.script_path("test-webkitperl")]


class MacPort(WebKitPort):

    @classmethod
    def name(cls):
        return "Mac"

    @classmethod
    def flag(cls):
        return "--port=mac"


class GtkPort(WebKitPort):

    @classmethod
    def name(cls):
        return "Gtk"

    @classmethod
    def flag(cls):
        return "--port=gtk"

    @classmethod
    def build_webkit_command(cls, build_style=None):
        command = WebKitPort.build_webkit_command(build_style=build_style)
        command.append("--gtk")
        command.append('--makeargs="-j%s"' % Executive.cpu_count())
        return command

    @classmethod
    def run_webkit_tests_command(cls):
        command = WebKitPort.run_webkit_tests_command()
        command.append("--gtk")
        return command


class QtPort(WebKitPort):

    @classmethod
    def name(cls):
        return "Qt"

    @classmethod
    def flag(cls):
        return "--port=qt"

    @classmethod
    def build_webkit_command(cls, build_style=None):
        command = WebKitPort.build_webkit_command(build_style=build_style)
        command.append("--qt")
        command.append('--makeargs="-j%s"' % Executive.cpu_count())
        return command


class ChromiumPort(WebKitPort):

    @classmethod
    def name(cls):
        return "Chromium"

    @classmethod
    def flag(cls):
        return "--port=chromium"

    @classmethod
    def update_webkit_command(cls):
        command = WebKitPort.update_webkit_command()
        command.append("--chromium")
        return command

    @classmethod
    def build_webkit_command(cls, build_style=None):
        command = WebKitPort.build_webkit_command(build_style=build_style)
        command.append("--chromium")
        return command

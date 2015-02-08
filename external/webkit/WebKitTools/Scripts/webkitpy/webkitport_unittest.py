#!/usr/bin/env python
import unittest

from webkitpy.executive import Executive
from webkitpy.webkitport import WebKitPort, MacPort, GtkPort, QtPort, ChromiumPort


class WebKitPortTest(unittest.TestCase):
    def test_mac_port(self):
        self.assertEquals(MacPort.name(), "Mac")
        self.assertEquals(MacPort.flag(), "--port=mac")
        self.assertEquals(MacPort.run_webkit_tests_command(), [WebKitPort.script_path("run-webkit-tests")])
        self.assertEquals(MacPort.build_webkit_command(), [WebKitPort.script_path("build-webkit")])
        self.assertEquals(MacPort.build_webkit_command(build_style="debug"), [WebKitPort.script_path("build-webkit"), "--debug"])
        self.assertEquals(MacPort.build_webkit_command(build_style="release"), [WebKitPort.script_path("build-webkit"), "--release"])

    def test_gtk_port(self):
        self.assertEquals(GtkPort.name(), "Gtk")
        self.assertEquals(GtkPort.flag(), "--port=gtk")
        self.assertEquals(GtkPort.run_webkit_tests_command(), [WebKitPort.script_path("run-webkit-tests"), "--gtk"])
        self.assertEquals(GtkPort.build_webkit_command(), [WebKitPort.script_path("build-webkit"), "--gtk", '--makeargs="-j%s"' % Executive.cpu_count()])
        self.assertEquals(GtkPort.build_webkit_command(build_style="debug"), [WebKitPort.script_path("build-webkit"), "--debug", "--gtk", '--makeargs="-j%s"' % Executive.cpu_count()])

    def test_qt_port(self):
        self.assertEquals(QtPort.name(), "Qt")
        self.assertEquals(QtPort.flag(), "--port=qt")
        self.assertEquals(QtPort.run_webkit_tests_command(), [WebKitPort.script_path("run-webkit-tests")])
        self.assertEquals(QtPort.build_webkit_command(), [WebKitPort.script_path("build-webkit"), "--qt", '--makeargs="-j%s"' % Executive.cpu_count()])
        self.assertEquals(QtPort.build_webkit_command(build_style="debug"), [WebKitPort.script_path("build-webkit"), "--debug", "--qt", '--makeargs="-j%s"' % Executive.cpu_count()])

    def test_chromium_port(self):
        self.assertEquals(ChromiumPort.name(), "Chromium")
        self.assertEquals(ChromiumPort.flag(), "--port=chromium")
        self.assertEquals(ChromiumPort.run_webkit_tests_command(), [WebKitPort.script_path("run-webkit-tests")])
        self.assertEquals(ChromiumPort.build_webkit_command(), [WebKitPort.script_path("build-webkit"), "--chromium"])
        self.assertEquals(ChromiumPort.build_webkit_command(build_style="debug"), [WebKitPort.script_path("build-webkit"), "--debug", "--chromium"])
        self.assertEquals(ChromiumPort.update_webkit_command(), [WebKitPort.script_path("update-webkit"), "--chromium"])


if __name__ == '__main__':
    unittest.main()

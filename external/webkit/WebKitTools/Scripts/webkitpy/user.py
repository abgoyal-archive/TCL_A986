import os
import shlex
import subprocess
import webbrowser

class User(object):
    @staticmethod
    def prompt(message, repeat=1, raw_input=raw_input):
        response = None
        while (repeat and not response):
            repeat -= 1
            response = raw_input(message)
        return response

    def edit(self, files):
        editor = os.environ.get("EDITOR") or "vi"
        args = shlex.split(editor)
        subprocess.call(args + files)

    def page(self, message):
        pager = os.environ.get("PAGER") or "less"
        try:
            child_process = subprocess.Popen([pager], stdin=subprocess.PIPE)
            child_process.communicate(input=message)
        except IOError, e:
            pass

    def confirm(self, message=None):
        if not message:
            message = "Continue?"
        response = raw_input("%s [Y/n]: " % message)
        return not response or response.lower() == "y"

    def open_url(self, url):
        webbrowser.open(url)

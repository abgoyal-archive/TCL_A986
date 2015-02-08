import os
import sys

def log(string):
    print >> sys.stderr, string

def error(string):
    log("ERROR: %s" % string)
    exit(1)

# Simple class to split output between multiple destinations
class tee:
    def __init__(self, *files):
        self.files = files

    def write(self, string):
        for file in self.files:
            file.write(string)

class OutputTee:
    def __init__(self):
        self._original_stdout = None
        self._original_stderr = None
        self._files_for_output = []

    def add_log(self, path):
        log_file = self._open_log_file(path)
        self._files_for_output.append(log_file)
        self._tee_outputs_to_files(self._files_for_output)
        return log_file

    def remove_log(self, log_file):
        self._files_for_output.remove(log_file)
        self._tee_outputs_to_files(self._files_for_output)
        log_file.close()

    @staticmethod
    def _open_log_file(log_path):
        (log_directory, log_name) = os.path.split(log_path)
        if log_directory and not os.path.exists(log_directory):
            os.makedirs(log_directory)
        return open(log_path, 'a+')

    def _tee_outputs_to_files(self, files):
        if not self._original_stdout:
            self._original_stdout = sys.stdout
            self._original_stderr = sys.stderr
        if files and len(files):
            sys.stdout = tee(self._original_stdout, *files)
            sys.stderr = tee(self._original_stderr, *files)
        else:
            sys.stdout = self._original_stdout
            sys.stderr = self._original_stderr

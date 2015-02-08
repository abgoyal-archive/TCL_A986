#!/usr/bin/env python
import os, re, sys, tickprocessor

class WindowsTickProcessor(tickprocessor.TickProcessor):

  def Unmangle(self, name):
    """Performs very simple unmangling of C++ names.

    Does not handle arguments and template arguments. The mangled names have
    the form:

       ?LookupInDescriptor@JSObject@internal@v8@@...arguments info...

    """
    # Name is mangled if it starts with a question mark.
    is_mangled = re.match("^\?(.*)", name)
    if is_mangled:
      substrings = is_mangled.group(1).split('@')
      try:
        # The function name is terminated by two @s in a row.  Find the
        # substrings that are part of the function name.
        index = substrings.index('')
        substrings = substrings[0:index]
      except ValueError:
        # If we did not find two @s in a row, the mangled name is not in
        # the format we expect and we give up.
        return name
      substrings.reverse()
      function_name = "::".join(substrings)
      return function_name
    return name


  def ParseMapFile(self, filename):
    """Parse map file and add symbol information to the cpp entries."""
    # Locate map file.
    has_dot = re.match('^([a-zA-F0-9_-]*)[\.]?.*$', filename)
    if has_dot:
      map_file_name = has_dot.group(1) + '.map'
      try:
        map_file = open(map_file_name, 'rb')
      except IOError:
        sys.exit("Could not open map file: " + map_file_name)
    else:
      sys.exit("Could not find map file for executable: " + filename)
    try:
      max_addr = 0
      min_addr = 2**30
      # Process map file and search for function entries.
      row_regexp = re.compile(' 0001:[0-9a-fA-F]{8}\s*([_\?@$0-9a-zA-Z]*)\s*([0-9a-fA-F]{8}).*')
      for line in map_file:
        row = re.match(row_regexp, line)
        if row:
          addr = int(row.group(2), 16)
          if addr > max_addr:
            max_addr = addr
          if addr < min_addr:
            min_addr = addr
          mangled_name = row.group(1)
          name = self.Unmangle(mangled_name)
          self.cpp_entries.Insert(addr, tickprocessor.CodeEntry(addr, name));
      i = min_addr
      # Mark the pages for which there are functions in the map file.
      while i < max_addr:
        page = i >> 12
        self.vm_extent[page] = 1
        i += 4096
    finally:
      map_file.close()


class WindowsCmdLineProcessor(tickprocessor.CmdLineProcessor):

  def __init__(self):
    super(WindowsCmdLineProcessor, self).__init__()
    self.binary_file = None

  def GetRequiredArgsNames(self):
    return 'binary log_file'

  def ProcessRequiredArgs(self, args):
    if len(args) != 2:
      self.PrintUsageAndExit()
    else:
      self.binary_file = args[0]
      self.log_file = args[1]


def Main():
  cmdline_processor = WindowsCmdLineProcessor()
  cmdline_processor.ProcessArguments()
  tickprocessor = WindowsTickProcessor()
  tickprocessor.ParseMapFile(cmdline_processor.binary_file)
  cmdline_processor.RunLogfileProcessing(tickprocessor)
  tickprocessor.PrintResults()

if __name__ == '__main__':
  Main()

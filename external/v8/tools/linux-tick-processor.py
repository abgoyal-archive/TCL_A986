#!/usr/bin/env python
import subprocess, re, sys, tickprocessor

class LinuxTickProcessor(tickprocessor.TickProcessor):

  def ParseVMSymbols(self, filename, start, end):
    """Extract symbols and add them to the cpp entries."""
    # Extra both dynamic and non-dynamic symbols.
    command = 'nm -C -n "%s"; nm -C -n -D "%s"' % (filename, filename)
    process = subprocess.Popen(command, shell=True,
                               stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
    pipe = process.stdout
    try:
      for line in pipe:
        row = re.match('^([0-9a-fA-F]{8}) . (.*)$', line)
        if row:
          addr = int(row.group(1), 16)
          if addr < start and addr < end - start:
            addr += start
          self.cpp_entries.Insert(addr, tickprocessor.CodeEntry(addr, row.group(2)))
    finally:
      pipe.close()


class LinuxCmdLineProcessor(tickprocessor.CmdLineProcessor):

  def GetRequiredArgsNames(self):
    return 'log_file'

  def ProcessRequiredArgs(self, args):
    if len(args) != 1:
      self.PrintUsageAndExit()
    else:
      self.log_file = args[0]


def Main():
  cmdline_processor = LinuxCmdLineProcessor()
  cmdline_processor.ProcessArguments()
  tick_processor = LinuxTickProcessor()
  cmdline_processor.RunLogfileProcessing(tick_processor)
  tick_processor.PrintResults()


if __name__ == '__main__':
  Main()

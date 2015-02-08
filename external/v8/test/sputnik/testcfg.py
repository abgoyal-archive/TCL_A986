import os
from os.path import join, exists
import sys
import test
import time


class SputnikTestCase(test.TestCase):

  def __init__(self, case, path, context, mode):
    super(SputnikTestCase, self).__init__(context, path)
    self.case = case
    self.mode = mode
    self.tmpfile = None
    self.source = None

  def IsNegative(self):
    return '@negative' in self.GetSource()

  def IsFailureOutput(self, output):
    if output.exit_code != 0:
      return True
    out = output.stdout
    return "SputnikError" in out

  def BeforeRun(self):
    self.tmpfile = sputnik.TempFile(suffix='.js', prefix='sputnik-', text=True)
    self.tmpfile.Write(self.GetSource())
    self.tmpfile.Close()

  def AfterRun(self):
    self.tmpfile.Dispose()
    self.tmpfile = None

  def GetCommand(self):
    result = [self.context.GetVm(self.mode)]
    result.append(self.tmpfile.name)
    return result

  def GetLabel(self):
    return "%s sputnik %s" % (self.mode, self.GetName())

  def GetName(self):
    return self.path[-1]

  def GetSource(self):
    if not self.source:
      self.source = self.case.GetSource()
    return self.source

class SputnikTestConfiguration(test.TestConfiguration):

  def __init__(self, context, root):
    super(SputnikTestConfiguration, self).__init__(context, root)

  def ListTests(self, current_path, path, mode):
    # Import the sputnik test runner script as a module
    testroot = join(self.root, 'sputniktests')
    modroot = join(testroot, 'tools')
    sys.path.append(modroot)
    import sputnik
    globals()['sputnik'] = sputnik
    test_suite = sputnik.TestSuite(testroot)
    test_suite.Validate()
    tests = test_suite.EnumerateTests([])
    result = []
    for test in tests:
      full_path = current_path + [test.GetPath()[-1]]
      if self.Contains(path, full_path):
        case = SputnikTestCase(test, full_path, self.context, mode)
        result.append(case)
    return result

  def GetBuildRequirements(self):
    return ['sample', 'sample=shell']

  def GetTestStatus(self, sections, defs):
    status_file = join(self.root, 'sputnik.status')
    if exists(status_file):
      test.ReadConfigurationInto(status_file, sections, defs)


def GetConfiguration(context, root):
  return SputnikTestConfiguration(context, root)

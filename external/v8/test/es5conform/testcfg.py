import test
import os
from os.path import join, exists


HARNESS_FILES = ['sth.js']


class ES5ConformTestCase(test.TestCase):

  def __init__(self, filename, path, context, root, mode, framework):
    super(ES5ConformTestCase, self).__init__(context, path)
    self.filename = filename
    self.mode = mode
    self.framework = framework
    self.root = root

  def IsNegative(self):
    return self.filename.endswith('-n.js')

  def GetLabel(self):
    return "%s es5conform %s" % (self.mode, self.GetName())

  def IsFailureOutput(self, output):
    if output.exit_code != 0:
      return True
    return 'FAILED!' in output.stdout

  def GetCommand(self):
    result = [self.context.GetVm(self.mode)]
    result += ['-e', 'var window = this']
    result += self.framework
    result.append(self.filename)
    result += ['-e', 'ES5Harness.startTesting()']
    return result

  def GetName(self):
    return self.path[-1]

  def GetSource(self):
    return open(self.filename).read()


class ES5ConformTestConfiguration(test.TestConfiguration):

  def __init__(self, context, root):
    super(ES5ConformTestConfiguration, self).__init__(context, root)

  def ListTests(self, current_path, path, mode):
    tests = []
    current_root = join(self.root, 'data', 'TestCases')
    harness = []
    harness += [join(self.root, 'data', 'SimpleTestHarness', f) for f in HARNESS_FILES]
    harness += [join(self.root, 'harness-adapt.js')]
    for root, dirs, files in os.walk(current_root):
      for dotted in [x  for x in dirs if x.startswith('.')]:
        dirs.remove(dotted)
      root_path = root[len(self.root):].split(os.path.sep)
      root_path = current_path + [x for x in root_path if x]
      for file in files:
        if file.endswith('.js'):
          full_path = root_path + [file[:-3]]
          full_path = [x for x in full_path if not (x in ['data', 'TestCases'])]
          if self.Contains(path, full_path):
            test = ES5ConformTestCase(join(root, file), full_path, self.context,
                                   self.root, mode, harness)
            tests.append(test)
    return tests

  def GetBuildRequirements(self):
    return ['sample', 'sample=shell']

  def GetTestStatus(self, sections, defs):
    status_file = join(self.root, 'es5conform.status')
    if exists(status_file):
      test.ReadConfigurationInto(status_file, sections, defs)


def GetConfiguration(context, root):
  return ES5ConformTestConfiguration(context, root)

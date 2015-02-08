#!/usr/bin/env python
"""Unit test for the gtest_xml_output module."""

__author__ = "keith.ray@gmail.com (Keith Ray)"

import os
from xml.dom import minidom, Node

import gtest_test_utils
import gtest_xml_test_utils


GTEST_OUTPUT_SUBDIR = "xml_outfiles"
GTEST_OUTPUT_1_TEST = "gtest_xml_outfile1_test_"
GTEST_OUTPUT_2_TEST = "gtest_xml_outfile2_test_"

EXPECTED_XML_1 = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="0" disabled="0" errors="0" time="*" name="AllTests">
  <testsuite name="PropertyOne" tests="1" failures="0" disabled="0" errors="0" time="*">
    <testcase name="TestSomeProperties" status="run" time="*" classname="PropertyOne" SetUpProp="1" TestSomeProperty="1" TearDownProp="1" />
  </testsuite>
</testsuites>
"""

EXPECTED_XML_2 = """<?xml version="1.0" encoding="UTF-8"?>
<testsuites tests="1" failures="0" disabled="0" errors="0" time="*" name="AllTests">
  <testsuite name="PropertyTwo" tests="1" failures="0" disabled="0" errors="0" time="*">
    <testcase name="TestSomeProperties" status="run" time="*" classname="PropertyTwo" SetUpProp="2" TestSomeProperty="2" TearDownProp="2" />
  </testsuite>
</testsuites>
"""


class GTestXMLOutFilesTest(gtest_xml_test_utils.GTestXMLTestCase):
  """Unit test for Google Test's XML output functionality."""

  def setUp(self):
    # We want the trailing '/' that the last "" provides in os.path.join, for
    # telling Google Test to create an output directory instead of a single file
    # for xml output.
    self.output_dir_ = os.path.join(gtest_test_utils.GetTempDir(),
                                    GTEST_OUTPUT_SUBDIR, "")
    self.DeleteFilesAndDir()

  def tearDown(self):
    self.DeleteFilesAndDir()

  def DeleteFilesAndDir(self):
    try:
      os.remove(os.path.join(self.output_dir_, GTEST_OUTPUT_1_TEST + ".xml"))
    except os.error:
      pass
    try:
      os.remove(os.path.join(self.output_dir_, GTEST_OUTPUT_2_TEST + ".xml"))
    except os.error:
      pass
    try:
      os.rmdir(self.output_dir_)
    except os.error:
      pass

  def testOutfile1(self):
    self._TestOutFile(GTEST_OUTPUT_1_TEST, EXPECTED_XML_1)

  def testOutfile2(self):
    self._TestOutFile(GTEST_OUTPUT_2_TEST, EXPECTED_XML_2)

  def _TestOutFile(self, test_name, expected_xml):
    gtest_prog_path = gtest_test_utils.GetTestExecutablePath(test_name)
    command = [gtest_prog_path, "--gtest_output=xml:%s" % self.output_dir_]
    p = gtest_test_utils.Subprocess(command,
                                    working_dir=gtest_test_utils.GetTempDir())
    self.assert_(p.exited)
    self.assertEquals(0, p.exit_code)

    # TODO(wan@google.com): libtool causes the built test binary to be
    #   named lt-gtest_xml_outfiles_test_ instead of
    #   gtest_xml_outfiles_test_.  To account for this possibillity, we
    #   allow both names in the following code.  We should remove this
    #   hack when Chandler Carruth's libtool replacement tool is ready.
    output_file_name1 = test_name + ".xml"
    output_file1 = os.path.join(self.output_dir_, output_file_name1)
    output_file_name2 = 'lt-' + output_file_name1
    output_file2 = os.path.join(self.output_dir_, output_file_name2)
    self.assert_(os.path.isfile(output_file1) or os.path.isfile(output_file2),
                 output_file1)

    expected = minidom.parseString(expected_xml)
    if os.path.isfile(output_file1):
      actual = minidom.parse(output_file1)
    else:
      actual = minidom.parse(output_file2)
    self.NormalizeXml(actual.documentElement)
    self.AssertEquivalentNodes(expected.documentElement,
                               actual.documentElement)
    expected.unlink()
    actual.unlink()


if __name__ == "__main__":
  os.environ["GTEST_STACK_TRACE_DEPTH"] = "0"
  gtest_test_utils.Main()

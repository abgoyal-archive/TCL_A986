#! /usr/bin/python
"""Tests python protocol buffers against the golden message.

Note that the golden messages exercise every known field type, thus this
test ends up exercising and verifying nearly all of the parsing and
serialization code in the whole library.

TODO(kenton):  Merge with wire_format_test?  It doesn't make a whole lot of
sense to call this a test of the "message" module, which only declares an
abstract interface.
"""

__author__ = 'gps@google.com (Gregory P. Smith)'

import unittest
from google.protobuf import unittest_import_pb2
from google.protobuf import unittest_pb2
from google.protobuf.internal import test_util


class MessageTest(unittest.TestCase):

  def testGoldenMessage(self):
    golden_data = test_util.GoldenFile('golden_message').read()
    golden_message = unittest_pb2.TestAllTypes()
    golden_message.ParseFromString(golden_data)
    test_util.ExpectAllFieldsSet(self, golden_message)
    self.assertTrue(golden_message.SerializeToString() == golden_data)

  def testGoldenExtensions(self):
    golden_data = test_util.GoldenFile('golden_message').read()
    golden_message = unittest_pb2.TestAllExtensions()
    golden_message.ParseFromString(golden_data)
    all_set = unittest_pb2.TestAllExtensions()
    test_util.SetAllExtensions(all_set)
    self.assertEquals(all_set, golden_message)
    self.assertTrue(golden_message.SerializeToString() == golden_data)

  def testGoldenPackedMessage(self):
    golden_data = test_util.GoldenFile('golden_packed_fields_message').read()
    golden_message = unittest_pb2.TestPackedTypes()
    golden_message.ParseFromString(golden_data)
    all_set = unittest_pb2.TestPackedTypes()
    test_util.SetAllPackedFields(all_set)
    self.assertEquals(all_set, golden_message)
    self.assertTrue(all_set.SerializeToString() == golden_data)

  def testGoldenPackedExtensions(self):
    golden_data = test_util.GoldenFile('golden_packed_fields_message').read()
    golden_message = unittest_pb2.TestPackedExtensions()
    golden_message.ParseFromString(golden_data)
    all_set = unittest_pb2.TestPackedExtensions()
    test_util.SetAllPackedExtensions(all_set)
    self.assertEquals(all_set, golden_message)
    self.assertTrue(all_set.SerializeToString() == golden_data)

if __name__ == '__main__':
  unittest.main()

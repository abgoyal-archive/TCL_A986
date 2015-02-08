#!/usr/bin/env python
import unittest

from webkitpy.mock import Mock
from webkitpy.patchcollection import PersistentPatchCollection, PersistentPatchCollectionDelegate


class TestPersistentPatchCollectionDelegate(PersistentPatchCollectionDelegate):
    def collection_name(self):
        return "test-collection"

    def fetch_potential_patch_ids(self):
        return [42, 192, 87]

    def status_server(self):
        return Mock()

    def is_terminal_status(self, status):
        return False


class PersistentPatchCollectionTest(unittest.TestCase):
    def test_next(self):
        collection = PersistentPatchCollection(TestPersistentPatchCollectionDelegate())
        collection.next()

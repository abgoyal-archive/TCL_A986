import unittest
from webkitpy.grammar import join_with_separators

class GrammarTest(unittest.TestCase):

    def test_join_with_separators(self):
        self.assertEqual(join_with_separators(["one", "two", "three"]), "one, two, and three")

if __name__ == '__main__':
    unittest.main()

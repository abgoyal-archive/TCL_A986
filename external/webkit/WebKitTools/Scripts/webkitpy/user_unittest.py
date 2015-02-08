import unittest
from webkitpy.user import User

class UserTest(unittest.TestCase):

    example_user_response = "example user response"

    def test_prompt_repeat(self):
        self.repeatsRemaining = 2
        def mock_raw_input(message):
            self.repeatsRemaining -= 1
            if not self.repeatsRemaining:
                return UserTest.example_user_response
            return None
        self.assertEqual(User.prompt("input", repeat=self.repeatsRemaining, raw_input=mock_raw_input), UserTest.example_user_response)

    def test_prompt_when_exceeded_repeats(self):
        self.repeatsRemaining = 2
        def mock_raw_input(message):
            self.repeatsRemaining -= 1
            return None
        self.assertEqual(User.prompt("input", repeat=self.repeatsRemaining, raw_input=mock_raw_input), None)

if __name__ == '__main__':
    unittest.main()

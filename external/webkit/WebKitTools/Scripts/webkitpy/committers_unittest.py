import unittest
from committers import CommitterList, Committer, Reviewer

class CommittersTest(unittest.TestCase):

    def test_committer_lookup(self):
        committer = Committer('Test One', 'one@test.com')
        reviewer = Reviewer('Test Two', ['two@test.com', 'two@rad.com', 'so_two@gmail.com'])
        committer_list = CommitterList(committers=[committer], reviewers=[reviewer])

        # Test valid committer and reviewer lookup
        self.assertEqual(committer_list.committer_by_email('one@test.com'), committer)
        self.assertEqual(committer_list.reviewer_by_email('two@test.com'), reviewer)
        self.assertEqual(committer_list.committer_by_email('two@test.com'), reviewer)
        self.assertEqual(committer_list.committer_by_email('two@rad.com'), reviewer)
        self.assertEqual(committer_list.reviewer_by_email('so_two@gmail.com'), reviewer)

        # Test that the first email is assumed to be the Bugzilla email address (for now)
        self.assertEqual(committer_list.committer_by_email('two@rad.com').bugzilla_email(), 'two@test.com')

        # Test that a known committer is not returned during reviewer lookup
        self.assertEqual(committer_list.reviewer_by_email('one@test.com'), None)

        # Test that unknown email address fail both committer and reviewer lookup
        self.assertEqual(committer_list.committer_by_email('bar@bar.com'), None)
        self.assertEqual(committer_list.reviewer_by_email('bar@bar.com'), None)

        # Test that emails returns a list.
        self.assertEqual(committer.emails, ['one@test.com'])

        # Test that committers returns committers and reviewers and reviewers() just reviewers.
        self.assertEqual(committer_list.committers(), [committer, reviewer])
        self.assertEqual(committer_list.reviewers(), [reviewer])


if __name__ == '__main__':
    unittest.main()

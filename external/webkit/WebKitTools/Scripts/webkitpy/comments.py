from webkitpy.changelogs import view_source_url


def bug_comment_from_svn_revision(svn_revision):
    return "Committed r%s: <%s>" % (svn_revision,
                                    view_source_url(svn_revision))


def bug_comment_from_commit_text(scm, commit_text):
    svn_revision = scm.svn_revision_from_commit_text(commit_text)
    return bug_comment_from_svn_revision(svn_revision)

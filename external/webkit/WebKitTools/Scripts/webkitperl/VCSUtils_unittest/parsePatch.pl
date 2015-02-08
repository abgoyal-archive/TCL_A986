#!/usr/bin/perl -w
use strict;
use warnings;

use Test::More;
use VCSUtils;

my @diffHashRefKeys = ( # The hash reference keys to check per diff.
    "copiedFromPath",
    "indexPath",
    "sourceRevision",
    "svnConvertedText",
);

# New test
my $testNameStart = "parsePatch(): [SVN: Rename] ";
my $patch = <<'END',
Index: Makefile
===================================================================
--- Makefile	(revision 53131)
+++ Makefile	(working copy)
@@ -1,1 +0,0 @@
-MODULES = JavaScriptCore JavaScriptGlue WebCore WebKit WebKitTools
Index: Makefile_new
===================================================================
--- Makefile_new	(revision 53131)	(from Makefile:53131)
+++ Makefile_new	(working copy)
@@ -0,0 +1,1 @@
+MODULES = JavaScriptCore JavaScriptGlue WebCore WebKit WebKitTools
END

my @expectedDiffHashRefs = (
{
    svnConvertedText => <<'END',
Index: Makefile
===================================================================
--- Makefile	(revision 53131)
+++ Makefile	(working copy)
@@ -1,1 +0,0 @@
-MODULES = JavaScriptCore JavaScriptGlue WebCore WebKit WebKitTools
END
    copiedFromPath => undef,
    indexPath => "Makefile",
    sourceRevision => "53131",
},
{
    svnConvertedText => <<'END',
Index: Makefile_new
===================================================================
--- Makefile_new	(revision 53131)	(from Makefile:53131)
+++ Makefile_new	(working copy)
@@ -0,0 +1,1 @@
+MODULES = JavaScriptCore JavaScriptGlue WebCore WebKit WebKitTools
END
    copiedFromPath => "Makefile",
    indexPath => "Makefile_new",
    sourceRevision => "53131",
},
);

plan(tests => @expectedDiffHashRefs * @diffHashRefKeys);

my $fileHandle;
open($fileHandle, "<", \$patch);

my @gotDiffHashRefs = parsePatch($fileHandle);

my $i = 0;
foreach my $expectedDiffHashRef (@expectedDiffHashRefs) {

    my $gotDiffHashRef = $gotDiffHashRefs[$i++];

    foreach my $diffHashRefKey (@diffHashRefKeys) {
        my $testName = "${testNameStart}[diff $i] key=\"$diffHashRefKey\"";
        is($gotDiffHashRef->{$diffHashRefKey}, $expectedDiffHashRef->{$diffHashRefKey}, $testName);
    }
}

#!/usr/bin/perl
use Test::Simple tests => 7;
use VCSUtils;

# The source ChangeLog for these tests is the following:
# 
# 2009-12-22  Alice  <alice@email.address>
# 
#         Reviewed by Ray.
# 
#         Changed some code on 2009-12-22.
# 
#         * File:
#         * File2:
# 
# 2009-12-21  Alice  <alice@email.address>
# 
#         Reviewed by Ray.
# 
#         Changed some code on 2009-12-21.
# 
#         * File:
#         * File2:

my $title;
my $in;
my $out;

# New test
$title = "fixChangeLogPatch: [no change] First line is new line.";

$in = <<'END';
--- ChangeLog
+++ ChangeLog
@@ -1,3 +1,11 @@
+2009-12-22  Bob  <bob@email.address>
+
+        Reviewed by Ray.
+
+        Changed some more code on 2009-12-22.
+
+        * File:
+
 2009-12-22  Alice  <alice@email.address>
 
         Reviewed by Ray.
END

ok(fixChangeLogPatch($in) eq $in, $title);

# New test
$title = "fixChangeLogPatch: [no change] No date string.";

$in = <<'END';
--- ChangeLog
+++ ChangeLog
@@ -6,6 +6,7 @@
 
         * File:
         * File2:
+        * File3:
 
 2009-12-21  Alice  <alice@email.address>
 
END

ok(fixChangeLogPatch($in) eq $in, $title);

# New test
$title = "fixChangeLogPatch: [no change] New entry inserted in middle.";

$in = <<'END';
--- ChangeLog
+++ ChangeLog
@@ -11,6 +11,14 @@
 
         Reviewed by Ray.
 
+        Changed some more code on 2009-12-21.
+
+        * File:
+
+2009-12-21  Alice  <alice@email.address>
+
+        Reviewed by Ray.
+
         Changed some code on 2009-12-21.
 
         * File:
END

ok(fixChangeLogPatch($in) eq $in, $title);

# New test
$title = "fixChangeLogPatch: Leading context includes first line.";

$in = <<'END';
--- ChangeLog
+++ ChangeLog
@@ -1,5 +1,13 @@
 2009-12-22  Alice  <alice@email.address>
 
+        Reviewed by Sue.
+
+        Changed some more code on 2009-12-22.
+
+        * File:
+
+2009-12-22  Alice  <alice@email.address>
+
         Reviewed by Ray.
 
         Changed some code on 2009-12-22.
END

$out = <<'END';
--- ChangeLog
+++ ChangeLog
@@ -1,3 +1,11 @@
+2009-12-22  Alice  <alice@email.address>
+
+        Reviewed by Sue.
+
+        Changed some more code on 2009-12-22.
+
+        * File:
+
 2009-12-22  Alice  <alice@email.address>
 
         Reviewed by Ray.
END

ok(fixChangeLogPatch($in) eq $out, $title);

# New test
$title = "fixChangeLogPatch: Leading context does not include first line.";

$in = <<'END';
@@ -2,6 +2,14 @@
 
         Reviewed by Ray.
 
+        Changed some more code on 2009-12-22.
+
+        * File:
+
+2009-12-22  Alice  <alice@email.address>
+
+        Reviewed by Ray.
+
         Changed some code on 2009-12-22.
 
         * File:
END

$out = <<'END';
@@ -1,3 +1,11 @@
+2009-12-22  Alice  <alice@email.address>
+
+        Reviewed by Ray.
+
+        Changed some more code on 2009-12-22.
+
+        * File:
+
 2009-12-22  Alice  <alice@email.address>
 
         Reviewed by Ray.
END

ok(fixChangeLogPatch($in) eq $out, $title);

# New test
$title = "fixChangeLogPatch: Non-consecutive line additions.";

# This can occur, for example, if the new ChangeLog entry includes
# trailing white space in the first blank line but not the second.
# A diff command can then match the second blank line of the new
# ChangeLog entry with the first blank line of the old.
# The svn diff command with the default --diff-cmd has done this.
$in = <<'END';
@@ -1,5 +1,11 @@
 2009-12-22  Alice  <alice@email.address>
+ <pretend-whitespace>
+        Reviewed by Ray.
 
+        Changed some more code on 2009-12-22.
+
+2009-12-22  Alice  <alice@email.address>
+
         Reviewed by Ray.
 
         Changed some code on 2009-12-22.
END

$out = <<'END';
@@ -1,3 +1,9 @@
+2009-12-22  Alice  <alice@email.address>
+ <pretend-whitespace>
+        Reviewed by Ray.
+
+        Changed some more code on 2009-12-22.
+
 2009-12-22  Alice  <alice@email.address>
 
         Reviewed by Ray.
END

ok(fixChangeLogPatch($in) eq $out, $title);

# New test
$title = "fixChangeLogPatch: Additional edits after new entry.";

$in = <<'END';
@@ -2,10 +2,17 @@
 
         Reviewed by Ray.
 
+        Changed some more code on 2009-12-22.
+
+        * File:
+
+2009-12-22  Alice  <alice@email.address>
+
+        Reviewed by Ray.
+
         Changed some code on 2009-12-22.
 
         * File:
-        * File2:
 
 2009-12-21  Alice  <alice@email.address>
 
END

$out = <<'END';
@@ -1,11 +1,18 @@
+2009-12-22  Alice  <alice@email.address>
+
+        Reviewed by Ray.
+
+        Changed some more code on 2009-12-22.
+
+        * File:
+
 2009-12-22  Alice  <alice@email.address>
 
         Reviewed by Ray.
 
         Changed some code on 2009-12-22.
 
         * File:
-        * File2:
 
 2009-12-21  Alice  <alice@email.address>
 
END

ok(fixChangeLogPatch($in) eq $out, $title);

#!/usr/bin/perl
use Test::Simple tests => 4;
use VCSUtils;

# Call a function while suppressing STDERR.
sub callSilently($@) {
    my ($func, @args) = @_;

    open(OLDERR, ">&STDERR");
    close(STDERR);
    my @returnValue = &$func(@args);
    open(STDERR, ">&OLDERR");
    close(OLDERR); # FIXME: Is this necessary?

    return @returnValue;
}

# New test
$title = "runPatchCommand: Unsuccessful patch, forcing.";

# Since $patch has no "Index:" path, passing this to runPatchCommand
# should not affect any files.
my $patch = <<'END';
Garbage patch contents
END

# We call via callSilently() to avoid output like the following to STDERR:
# patch: **** Only garbage was found in the patch input.
$argsHashRef = {ensureForce => 1};
$exitStatus = callSilently(\&runPatchCommand, $patch, ".", "file_to_patch.txt", $argsHashRef);

ok($exitStatus != 0, $title);

# New test
$title = "runPatchCommand: New file, --dry-run.";

# This file should not exist after the tests, but we take care with the
# file name and contents just in case.
my $fileToPatch = "temp_OK_TO_ERASE__README_FOR_MORE.txt";
$patch = <<END;
Index: $fileToPatch
===================================================================
--- $fileToPatch	(revision 0)
+++ $fileToPatch	(revision 0)
@@ -0,0 +1,5 @@
+This is a test file for WebKitTools/Scripts/VCSUtils_unittest.pl.
+This file should not have gotten created on your system.
+If it did, some unit tests don't seem to be working quite right:
+It would be great if you could file a bug report. Thanks!
+---------------------------------------------------------------------
END

# --dry-run prevents creating any files.
# --silent suppresses the success message to STDOUT.
$argsHashRef = {options => ["--dry-run", "--silent"]};
$exitStatus = runPatchCommand($patch, ".", $fileToPatch, $argsHashRef);

ok($exitStatus == 0, $title);

# New test
$title = "runPatchCommand: New file: \"$fileToPatch\".";

$argsHashRef = {options => ["--silent"]};
$exitStatus = runPatchCommand($patch, ".", $fileToPatch, $argsHashRef);

ok($exitStatus == 0, $title);

# New test
$title = "runPatchCommand: Reverse new file (clean up previous).";

$argsHashRef = {shouldReverse => 1,
                options => ["--silent", "--remove-empty-files"]}; # To clean up.
$exitStatus = runPatchCommand($patch, ".", $fileToPatch, $argsHashRef);
ok($exitStatus == 0, $title);

#!/usr/bin/perl
use Test::Simple tests => 10;
use VCSUtils;

# New test
$title = "generatePatchCommand: Undefined optional arguments.";

my $argsHashRef;
my ($patchCommand, $isForcing) = VCSUtils::generatePatchCommand($argsHashRef);

ok($patchCommand eq "patch -p0", $title);
ok($isForcing == 0, $title);

# New test
$title = "generatePatchCommand: Undefined options.";

my $options;
$argsHashRef = {options => $options};
($patchCommand, $isForcing) = VCSUtils::generatePatchCommand($argsHashRef);

ok($patchCommand eq "patch -p0", $title);
ok($isForcing == 0, $title);

# New test
$title = "generatePatchCommand: --force and no \"ensure force\".";

$argsHashRef = {options => ["--force"]};
($patchCommand, $isForcing) = VCSUtils::generatePatchCommand($argsHashRef);

ok($patchCommand eq "patch -p0 --force", $title);
ok($isForcing == 1, $title);

# New test
$title = "generatePatchCommand: no --force and \"ensure force\".";

$argsHashRef = {ensureForce => 1};
($patchCommand, $isForcing) = VCSUtils::generatePatchCommand($argsHashRef);

ok($patchCommand eq "patch -p0 --force", $title);
ok($isForcing == 1, $title);

# New test
$title = "generatePatchCommand: \"should reverse\".";

$argsHashRef = {shouldReverse => 1};
($patchCommand, $isForcing) = VCSUtils::generatePatchCommand($argsHashRef);

ok($patchCommand eq "patch -p0 --reverse", $title);

# New test
$title = "generatePatchCommand: --fuzz=3, --force.";

$argsHashRef = {options => ["--fuzz=3", "--force"]};
($patchCommand, $isForcing) = VCSUtils::generatePatchCommand($argsHashRef);

ok($patchCommand eq "patch -p0 --force --fuzz=3", $title);

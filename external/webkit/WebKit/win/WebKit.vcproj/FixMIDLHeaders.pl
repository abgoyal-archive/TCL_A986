#! /usr/bin/perl -w
use File::Find;
use strict;
use warnings;

my $dir = $ARGV[0];

$dir = `cygpath -u '$dir'`;
chomp($dir);

find(\&finder, $dir);

sub finder
{
    my $fileName = $_;

    return unless ($fileName =~ /IGEN_DOM(.*)\.h/);

    open(IN, "<", $fileName);
    my @contents = <IN>;
    close(IN);

    open(OUT, ">", $fileName);

    my $state = 0;
    foreach my $line (@contents) {
        if ($line =~ /^\/\* header files for imported files \*\//) {
            $state = 1;
        } elsif ($line =~ /^#include "oaidl\.h"/) {
            die "#include \"oaidl.h\" did not come second" if $state != 1;
            $state = 2;
        } elsif ($line =~ /^#include "ocidl\.h"/) {
            die "#include \"ocidl.h\" did not come third" if $state != 2;
            $state = 3;
        } elsif ($line =~ /^#include "IGEN_DOM/ && $state == 3) {
            $state = 4;
        } elsif ($line =~ /^#include "(IGEN_DOM.*)\.h"/ && $state == 4) {
            next;
        }

        print OUT $line;
    }

    close(OUT);
}

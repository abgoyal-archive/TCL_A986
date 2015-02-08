#! /usr/bin/perl
use strict;
use warnings;

my $grammar = $ARGV[0];
my $fileBase = $ARGV[1];

system("bison -d -p cssyy " . $grammar . " -o " . $fileBase . ".tab.c");

open HEADER, ">" . $fileBase . ".h" or die;
print HEADER << "EOF";
#ifndef CSSGRAMMAR_H
#define CSSGRAMMAR_H
EOF

open HPP, "<" . $fileBase . ".tab.h" or die;
while (<HPP>) {
    print HEADER;
}
close HPP;

print HEADER "#endif\n";

close HEADER;

unlink($fileBase . ".tab.h");

open CPP, ">" . $fileBase . ".cpp" or die;
open GENSRC, "<" . $fileBase . ".tab.c" or die;
while (<GENSRC>) {
    print CPP;
}
close GENSRC;
close CPP;

unlink($fileBase . ".tab.c");

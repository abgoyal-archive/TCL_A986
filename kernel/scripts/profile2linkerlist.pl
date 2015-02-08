#!/usr/bin/perl
use strict;

while (<>) {
  my $line = $_;

  $_ =~ /\W*[0-9]+\W*([a-zA-Z\_0-9]+)\W*[0-9]+/;

  print "*(.text.$1)\n"
      unless ($line =~ /unknown/) || ($line =~ /total/);
}

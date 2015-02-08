#!/usr/bin/perl -w
use strict;
use Getopt::Long;

my $preprocessor;

GetOptions('preprocessor=s' => \$preprocessor);

if (!$preprocessor) {
    $preprocessor = "/usr/bin/gcc -E -P -x c++";
}

my $header = $ARGV[0];
shift;

my $out = $ARGV[0];
shift;

open HEADER, ">", $header or die;
open OUT, ">", $out or die;

print HEADER "namespace WebCore {\n";
print OUT "namespace WebCore {\n";

for my $in (@ARGV) {
    $in =~ /(\w+)\.css$/ or die;
    my $name = $1;

    # Slurp in the CSS file.
    open IN, $preprocessor . " " . $in . "|" or die;
    my $text; { local $/; $text = <IN>; }
    close IN;

    # Remove comments in a simple-minded way that will work fine for our files.
    # Could do this a fancier way if we were worried about arbitrary CSS source.
    $text =~ s|/\*.*?\*/||gs;

    # Crunch whitespace just to make it a little smaller.
    # Could do work to avoid doing this inside quote marks but our files don't have runs of spaces in quotes.
    # Could crunch further based on places where whitespace is optional.
    $text =~ s|\s+| |gs;
    $text =~ s|^ ||;
    $text =~ s| $||;

    # Write out a C array of the characters.
    my $length = length $text;
    print HEADER "extern const char ${name}UserAgentStyleSheet[${length}];\n";
    print OUT "extern const char ${name}UserAgentStyleSheet[${length}] = {\n";
    my $i = 0;
    while ($i < $length) {
        print OUT "    ";
        my $j = 0;
        while ($j < 16 && $i < $length) {
            print OUT ", " unless $j == 0;
            print OUT ord substr $text, $i, 1;
            ++$i;
            ++$j;
        }
        print OUT "," unless $i == $length;
        print OUT "\n";
    }
    print OUT "};\n";

}

print HEADER "}\n";
print OUT "}\n";

#!/usr/bin/perl -w
use strict;

package InFilesParser;

my $isParsingCommonParameters;
my $hasStartedParsing;

# Helper functions

sub trimComment
{   
    my $string = shift;
    $string =~ s/#.+$//;
    chomp($string);
    return $string;
}

sub trimWS
{
    my $string = shift;
    $string =~ s/^\s+//;
    $string =~ s/\s+$//;
    chomp($string);
    return $string;
}

sub trimQuoteAndWS
{
    my $string = shift;
    $string =~ s/\"([^\"]+)\"/$1/;
    return trimWS($string);
}

# Default constructor

sub new
{
    my $object = shift;
    my $reference = { };

    # Initialize the parser.
    $isParsingCommonParameters = 1;
    $hasStartedParsing = 0;

    bless($reference, $object);
    return $reference;
}

# parse take 3 attributes:
# - the filestream to read from (the caller has to open / close it).
# - the commonParameterHandler called when parsing the first part of the file with the parameter and the value.
# - the perTagHandler called for each optional parameter with the element name, the parameter and its value.
#   If no parameter were provided, it is called once with an empty parameter and value.
sub parse($)
{
    my $object = shift;
    my $fileStream = shift; # IO::File only
    my $commonParameterHandler = shift;
    my $perTagHandler = shift;

    foreach (<$fileStream>) {
        # Empty line, change from common parameter part
        # to per tag part if we have started parsing.
        if (/^$/) {
            if ($hasStartedParsing) {
                $isParsingCommonParameters = 0;
            }
            next;
        }

        # There may be a few empty lines at the beginning of the file
        # so detect the first non empty line which starts the common
        # parameters part.
        $hasStartedParsing = 1;

        if (/^#/) {
            next;
        }

        $_ = trimComment($_);

        if ($isParsingCommonParameters) {
            my ($name, $value) = split '=', $_;

            $name = trimWS($name);
            if (defined($value)) {
                $value = trimQuoteAndWS($value);
            } else {
                # We default to 1 as it eases the syntax.
                $value = "1";
            }

            &$commonParameterHandler($name, $value);
        } else {
            # Parsing per-tag parameters.

            # Split the tag name ($1) from the optionnal parameter(s) ($2)
            /^(\S+)\s*(.*)$/;
            my $elementName = $1;

            if ($2) {
                my @options = split "," , $2;
                my ($option, $value);
                for (my $i = 0; $i < @options; ++$i) {
                    ($option, $value) = split "=", $options[$i];
                    $option = trimWS($option);
                    if (defined($value)) {
                        $value = trimQuoteAndWS($value);
                    } else {
                        # We default to 1 as it eases the syntax.
                        $value = "1";
                    }

                    &$perTagHandler($elementName, $option, $value);
                }
            } else {
                # No parameter was given so call it with empty strings.
                &$perTagHandler($elementName, "", "");
            }
        }
    }
}

1;

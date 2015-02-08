#!/usr/bin/perl -w
BEGIN {
   use Exporter   ();
   our ($VERSION, @ISA, @EXPORT, @EXPORT_OK, %EXPORT_TAGS);
   $VERSION     = 1.00;
   @ISA         = qw(Exporter);
   @EXPORT      = qw(&resetSpacingHeuristics &isOnlyWhiteSpace &applySpacingHeuristicsAndPrint &setPreviousAllowedLine &setPreviousAllowedLine &printPendingEmptyLines &ignoringLine);
   %EXPORT_TAGS = ();
   @EXPORT_OK   = ();
}

our @EXPORT_OK;

my $justFoundEmptyLine = 0;
my $previousLineWasDisallowed = 0;
my $previousAllowedLine = "";
my $pendingEmptyLines = "";

sub resetSpacingHeuristics
{
    $justFoundEmptyLine = 0;
    $previousLineWasDisallowed = 0;
    $previousAllowedLine = "";
    $pendingEmptyLines = "";
}

sub isOnlyWhiteSpace
{
    my $line = shift;
    my $isOnlyWhiteSpace = ($line =~ m/^\s+$/);
    $pendingEmptyLines .= $line if ($isOnlyWhiteSpace);
    return $isOnlyWhiteSpace;
}

sub applySpacingHeuristicsAndPrint
{
    my ($out, $line) = @_;
    
    printPendingEmptyLines($out, $line);
    $previousLineWasDisallowed = 0;
    print $out $line;
}

sub setPreviousAllowedLine
{
    my $line = shift;
    $previousAllowedLine = $line;
}

sub printPendingEmptyLines
{
    my $out = shift;
    my $line = shift;
    if ($previousLineWasDisallowed) {
        if (!($pendingEmptyLines eq "") && !($previousAllowedLine =~ m/{\s*$/) && !($line =~ m/^\s*}/)) {
            $pendingEmptyLines = "\n";
        } else {
            $pendingEmptyLines = "";
        }
    }
    print $out $pendingEmptyLines;
    $pendingEmptyLines = "";
}

sub ignoringLine
{
    # my $line = shift; # ignoring input argument
    $previousLineWasDisallowed = 1;
}

1;
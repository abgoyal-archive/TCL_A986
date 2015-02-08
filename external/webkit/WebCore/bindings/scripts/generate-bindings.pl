#!/usr/bin/perl -w
use strict;

use File::Path;
use Getopt::Long;
use Cwd;

use IDLParser;
use CodeGenerator;

my @idlDirectories;
my $outputDirectory;
my $generator;
my $defines;
my $preprocessor;
my $writeDependencies;

GetOptions('include=s@' => \@idlDirectories,
           'outputDir=s' => \$outputDirectory,
           'generator=s' => \$generator,
           'defines=s' => \$defines,
           'preprocessor=s' => \$preprocessor,
           'write-dependencies' => \$writeDependencies);

my $idlFile = $ARGV[0];

die('Must specify input file.') unless defined($idlFile);
die('Must specify IDL search path.') unless @idlDirectories;
die('Must specify generator') unless defined($generator);
die('Must specify output directory.') unless defined($outputDirectory);
die('Must specify defines') unless defined($defines);

$defines =~ s/^\s+|\s+$//g; # trim whitespace

# Parse the given IDL file.
my $parser = IDLParser->new(1);
my $document = $parser->Parse($idlFile, $defines, $preprocessor);

# Generate desired output for given IDL file.
my $codeGen = CodeGenerator->new(\@idlDirectories, $generator, $outputDirectory, 0, $preprocessor, $writeDependencies);
$codeGen->ProcessDocument($document, $defines);

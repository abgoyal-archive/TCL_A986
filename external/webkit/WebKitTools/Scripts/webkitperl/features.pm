use strict;
use warnings;

BEGIN {
   use Exporter   ();
   our ($VERSION, @ISA, @EXPORT, @EXPORT_OK, %EXPORT_TAGS);
   $VERSION     = 1.00;
   @ISA         = qw(Exporter);
   @EXPORT      = qw(&checkWebCoreFeatureSupport
                     &removeLibraryDependingOnFeature);
   %EXPORT_TAGS = ( );
   @EXPORT_OK   = ();
}

sub libraryContainsSymbol($$)
{
    my ($path, $symbol) = @_;

    if (isCygwin() or isWindows()) {
        # FIXME: Implement this for Windows.
        return 0;
    }

    my $foundSymbol = 0;
    if (-e $path) {
        open NM, "-|", "nm", $path or die;
        while (<NM>) {
            $foundSymbol = 1 if /$symbol/; # FIXME: This should probably check for word boundaries before/after the symbol name.
        }
        close NM;
    }
    return $foundSymbol;
}

sub hasFeature($$)
{
    my ($featureName, $path) = @_;
    my %symbolForFeature = (
        "MathML" => "MathMLElement",
        "SVG" => "SVGDefsElement", # We used to look for SVGElement but isSVGElement exists (and would match) in --no-svg builds.
        "Accelerated Compositing" => "GraphicsLayer",
        "3D Rendering" => "WebCoreHas3DRendering",
        "3D Canvas" => "WebGLShader",
        "WML" => "WMLElement",
        "XHTMLMP" => "isXHTMLMPDocument",
    );
    my $symbolName = $symbolForFeature{$featureName};
    die "Unknown feature: $featureName" unless $symbolName;
    return libraryContainsSymbol($path, $symbolName);
}

sub checkWebCoreFeatureSupport($$)
{
    my ($feature, $required) = @_;
    my $libraryName = "WebCore";
    my $path = builtDylibPathForName($libraryName);
    my $hasFeature = hasFeature($feature, $path);
    if ($required && !$hasFeature) {
        die "$libraryName at \"$path\" does not include $hasFeature support.  See build-webkit --help\n";
    }
    return $hasFeature;
}

sub removeLibraryDependingOnFeature($$$)
{
    my ($libraryName, $featureName, $shouldHaveFeature) = @_;
    my $path = builtDylibPathForName($libraryName);
    return unless -x $path;

    my $hasFeature = hasFeature($featureName, $path);
    system "rm -f $path" if ($shouldHaveFeature xor $hasFeature);
}

1;

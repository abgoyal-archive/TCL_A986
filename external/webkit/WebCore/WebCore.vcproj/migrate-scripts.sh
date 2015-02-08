#!/usr/bin/bash
NUMCPUPATH="../../WebKitTools/Scripts/num-cpus"
if [ -x $NUMCPUPATH ]; then
    NUMCPUS=`$NUMCPUPATH`
else
    NUMCPUS=1
fi

mkdir -p "$1"

XSRCROOT="`pwd`/.."
XSRCROOT=`realpath "$XSRCROOT"`
# Do a little dance to get the path into 8.3 form to make it safe for gnu make
# http://bugzilla.opendarwin.org/show_bug.cgi?id=8173
XSRCROOT=`cygpath -m -s "$XSRCROOT"`
XSRCROOT=`cygpath -u "$XSRCROOT"`
export XSRCROOT
export SOURCE_ROOT=$XSRCROOT

XDSTROOT="$1"
export XDSTROOT
# Do a little dance to get the path into 8.3 form to make it safe for gnu make
# http://bugzilla.opendarwin.org/show_bug.cgi?id=8173
XDSTROOT=`cygpath -m -s "$XDSTROOT"`
XDSTROOT=`cygpath -u "$XDSTROOT"`
export XDSTROOT

export WEBCORE=$XSRCROOT
export WEBKIT_OUTPUT=$XDSTROOT

make -f "$WEBCORE/WebCore.vcproj/MigrateScripts" -j ${NUMCPUS} || exit 1

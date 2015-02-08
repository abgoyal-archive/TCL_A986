#!/usr/bin/bash
if [ "$2" = "cairo" ]; then
    FeatureDefines=$1/tools/vsprops/FeatureDefinesCairo.vsprops
else
    FeatureDefines=$1/tools/vsprops/FeatureDefines.vsprops
fi

grep Value=\"ENABLE_ $FeatureDefines | sed 's/^.*Value\=\"/ /' | sed 's/\"//' | tr -d '\n'

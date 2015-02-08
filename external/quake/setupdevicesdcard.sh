#!/bin/bash
function gettop
{
    echo $TOP
}

T=$(gettop)
if [ -z "$T" ] ; then
    echo "please run your envsetup.sh script"
    exit 1
fi

echo "top found at $T"

echo "Creating Quake directories on the device's sdcard"

adb shell mkdir /sdcard
adb shell mkdir /sdcard/data
adb shell mkdir /sdcard/data/quake
adb shell mkdir /sdcard/data/quake/id1

echo "Copying Quake data files to the device. (This could take several minutes)"
adb push $T/external/quake/quake/app/id1 /sdcard/data/quake/id1
echo "Done."

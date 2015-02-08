#!/bin/bash
function gettop
{
    TOPFILE=config/envsetup.make
    if [ -n "$TOP" -a -f "$TOP/$TOPFILE" ] ; then
        echo $TOP
    else
        if [ -f $TOPFILE ] ; then
            echo $PWD
        else
            HERE=$PWD
            T=
            while [ \( ! \( -f $TOPFILE \) \) -a \( $PWD != "/" \) ]; do
                cd ..
                T=$PWD
            done
            cd $HERE
            if [ -f "$T/$TOPFILE" ]; then
                echo $T
            fi
        fi
    fi
}

T=$(gettop)
if [ -z "$T" ] ; then
    echo "please run your envsetup.sh script"
    exit 1
fi

echo "top found at $T"

echo "Removing Quake files and directories from the device's sdcard"

adb shell rm /sdcard/data/quake/id1/glQuake/*
adb shell rm /sdcard/data/quake/id1/glQuake/.DS_Store
adb shell rmdir /sdcard/data/quake/glQuake
adb shell rm /sdcard/data/quake/id1/*
adb shell rm /sdcard/data/quake/id1/.DS_Store
adb shell rmdir /sdcard/data/quake/id1
adb shell rm /sdcard/data/quake/.DS_Store
adb shell rmdir /sdcard/data/quake

echo "Done."
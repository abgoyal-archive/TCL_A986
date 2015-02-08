#!/bin/sh
set -e

# User may have a custom install script

if [ -x ~/bin/${INSTALLKERNEL} ]; then exec ~/bin/${INSTALLKERNEL} "$@"; fi
if [ -x /sbin/${INSTALLKERNEL} ]; then exec /sbin/${INSTALLKERNEL} "$@"; fi

# Default install

# this should work for both the pSeries zImage and the iSeries vmlinux.sm
image_name=`basename $2`

if [ -f $4/$image_name ]; then
	mv $4/$image_name $4/$image_name.old
fi

if [ -f $4/System.map ]; then
	mv $4/System.map $4/System.old
fi

cat $2 > $4/$image_name
cp $3 $4/System.map

# Copy all the bootable image files
path=$4
shift 4
while [ $# -ne 0 ]; do
	image_name=`basename $1`
	if [ -f $path/$image_name ]; then
		mv $path/$image_name $path/$image_name.old
	fi
	cat $1 > $path/$image_name
	shift
done;

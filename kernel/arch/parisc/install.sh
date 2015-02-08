#!/bin/sh
if [ -x ~/bin/${INSTALLKERNEL} ]; then exec ~/bin/${INSTALLKERNEL} "$@"; fi
if [ -x /sbin/${INSTALLKERNEL} ]; then exec /sbin/${INSTALLKERNEL} "$@"; fi

# Default install

if [ -f $4/vmlinux ]; then
	mv $4/vmlinux $4/vmlinux.old
fi

if [ -f $4/System.map ]; then
	mv $4/System.map $4/System.old
fi

cat $2 > $4/vmlinux
cp $3 $4/System.map

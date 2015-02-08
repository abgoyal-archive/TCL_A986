#!/bin/sh
if [ -x /sbin/${INSTALLKERNEL} ]; then
  exec /sbin/${INSTALLKERNEL} "$@"
fi

if [ "$2" = "zImage" ]; then
# Compressed install
  echo "Installing compressed kernel"
  if [ -f $4/vmlinuz-$1 ]; then
    mv $4/vmlinuz-$1 $4/vmlinuz.old
  fi

  if [ -f $4/System.map-$1 ]; then
    mv $4/System.map-$1 $4/System.old
  fi

  cat $2 > $4/vmlinuz-$1
  cp $3 $4/System.map-$1
else
# Normal install
  echo "Installing normal kernel"
  if [ -f $4/vmlinux-$1 ]; then
    mv $4/vmlinux-$1 $4/vmlinux.old
  fi

  if [ -f $4/System.map ]; then
    mv $4/System.map $4/System.old
  fi

  cat $2 > $4/vmlinux-$1
  cp $3 $4/System.map
fi

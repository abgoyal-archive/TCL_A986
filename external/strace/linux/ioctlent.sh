#! /bin/sh
case $# in
1)
	dir="$1"
	asm=asm
	;;
2)
	dir="$1"
	asm="$2"
	;;
*)
        echo "usage: $0 include-directory [asm-subdirectory]" >&2
        exit 1
	;;
esac

lookup_ioctls()
{
	type="$1"
	shift

	# Build the list of all ioctls
	regexp='^[[:space:]]*#[[:space:]]*define[[:space:]]\+[A-Z][A-Z0-9_]*[[:space:]]\+0x'"$type"'..\>'
	(cd "$dir" ; grep "$regexp" "$@" /dev/null 2>/dev/null ) |
		sed -ne "s,$asm/,asm/,g"'
s/^\(.*\):[[:space:]]*#[[:space:]]*define[[:space:]]*\([A-Z0-9_]*\)[[:space:]]*\(0x'"$type"'..\).*/	{ "\1",	"\2",	\3	},/p' \
		>> ioctls.h
}

: > ioctls.h
lookup_ioctls 22 scsi/sg.h
lookup_ioctls 46 linux/fb.h
lookup_ioctls 4B linux/kd.h
lookup_ioctls 53 linux/cdrom.h scsi/scsi.h scsi/scsi_ioctl.h
lookup_ioctls 54 $asm/ioctls.h
lookup_ioctls 56 linux/vt.h
lookup_ioctls '7[12]' linux/videotext.h
lookup_ioctls 89 $asm/sockios.h linux/sockios.h
lookup_ioctls 8B linux/wireless.h

files="linux/* $asm/* scsi/* sound/*"

# Build the list of all ioctls
regexp='^[[:space:]]*#[[:space:]]*define[[:space:]]\+[A-Z][A-Z0-9_]*[[:space:]]\+_S\?\(IO\|IOW\|IOR\|IOWR\)\>'
(cd $dir ; grep $regexp $files 2>/dev/null ) | \
	sed -ne "s,$asm/,asm/,g"'
s/^\(.*\):[[:space:]]*#[[:space:]]*define[[:space:]]*\([A-Z0-9_]*\)[[:space:]]*_S\?I.*(\([^[,]*\)[[:space:]]*,[[:space:]]*\([^,)]*\).*/	{ "\1",	"\2",	_IOC(_IOC_NONE,\3,\4,0)	},/p' \
	>> ioctls.h

# Some use a special base to offset their ioctls on. Extract that as well.
: > ioctldefs.h

bases=$(sed -ne 's/.*_IOC_NONE.*,[[:space:]]*\([A-Z][A-Z0-9_]\+\)[[:space:]+,].*/\1/p' ioctls.h | uniq | sort)
for base in $bases ; do
	echo "Looking for $base"
	regexp="^[[:space:]]*#[[:space:]]*define[[:space:]]\+$base"
	(cd $dir ; grep -h $regexp 2>/dev/null $files ) | \
		grep -v '\<_IO' >> ioctldefs.h
done

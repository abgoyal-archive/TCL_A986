#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile10.bz2 > testfile10 2>/dev/null || exit 0

./ecp testfile10 testfile10.tmp

cmp testfile10 testfile10.tmp

rm -f testfile10 testfile10.tmp

exit 0

#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile2.bz2 > testfile2 2>/dev/null || exit 0

./ecp testfile2 testfile2.tmp

rm -f testfile2 testfile2.tmp

exit 0

#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile8.bz2 > testfile8 2>/dev/null || exit 0

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile9.bz2 > testfile9 2>/dev/null || exit 0

LD_LIBRARY_PATH=../libebl:../libelf${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH \
  ../src/strip -o testfile.temp testfile8

cmp testfile9 testfile.temp

# Check elflint and the expected result.
LD_LIBRARY_PATH=../libebl:../libelf${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH \
  ../src/elflint -q testfile.temp

rm -f testfile8 testfile9 testfile.temp

exit 0

#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile12.bz2 > testfile12 2>/dev/null || exit 0

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile13.bz2 > testfile13 2>/dev/null || exit 0

LD_LIBRARY_PATH=../libebl:../libelf${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH \
  ../src/strip -o testfile.temp testfile12

cmp testfile13 testfile.temp

# Check elflint and the expected result.
LD_LIBRARY_PATH=../libebl:../libelf${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH \
  ../src/elflint -q testfile.temp

rm -f testfile12 testfile13 testfile.temp

exit 0

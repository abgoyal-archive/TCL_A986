#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile11.bz2 > testfile11 2>/dev/null || exit 0

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile7.bz2 > testfile7 2>/dev/null || exit 0

LD_LIBRARY_PATH=../libebl:../libelf${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH \
  ../src/strip -o testfile.temp testfile11

cmp testfile7 testfile.temp

# Check elflint and the expected result.
LD_LIBRARY_PATH=../libebl:../libelf${LD_LIBRARY_PATH:+:}$LD_LIBRARY_PATH \
  ../src/elflint -q testfile.temp

rm -f testfile11 testfile7 testfile.temp

exit 0

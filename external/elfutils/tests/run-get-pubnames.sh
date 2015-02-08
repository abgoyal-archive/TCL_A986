#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile.bz2 > testfile 2>/dev/null || exit 0

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile2.bz2 > testfile2 2>/dev/null || exit 0

./get-pubnames testfile testfile2 > get-pubnames.out

diff -u get-pubnames.out - <<"EOF"
 [ 0] "main", die: 104, cu: 11
CU name: "m.c"
object name: "main"
 [ 1] "a", die: 174, cu: 11
CU name: "m.c"
object name: "a"
 [ 2] "bar", die: 295, cu: 202
CU name: "b.c"
object name: "bar"
 [ 3] "foo", die: 5721, cu: 5628
CU name: "f.c"
object name: "foo"
 [ 0] "bar", die: 72, cu: 11
CU name: "b.c"
object name: "bar"
 [ 1] "foo", die: 2490, cu: 2429
CU name: "f.c"
object name: "foo"
 [ 2] "main", die: 2593, cu: 2532
CU name: "m.c"
object name: "main"
 [ 3] "a", die: 2663, cu: 2532
CU name: "m.c"
object name: "a"
EOF

rm -f testfile testfile2 get-pubnames.out

exit 0

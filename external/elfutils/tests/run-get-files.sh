#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile.bz2 > testfile 2>/dev/null || exit 0

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile2.bz2 > testfile2 2>/dev/null || exit 0

./get-files testfile testfile2 > get-files.out

diff -u get-files.out - <<"EOF"
cuhl = 11, o = 0, asz = 4, osz = 4, ncu = 191
 file[0] = "???"
 file[1] = "/home/drepper/gnu/new-bu/build/ttt/m.c"
cuhl = 11, o = 114, asz = 4, osz = 4, ncu = 5617
 file[0] = "???"
 file[1] = "/home/drepper/gnu/new-bu/build/ttt/b.c"
 file[2] = "/usr/lib/gcc-lib/i386-redhat-linux/2.96/include/stddef.h"
 file[3] = "/usr/lib/gcc-lib/i386-redhat-linux/2.96/include/stdarg.h"
 file[4] = "/usr/include/bits/types.h"
 file[5] = "/usr/include/bits/sched.h"
 file[6] = "/usr/include/bits/pthreadtypes.h"
 file[7] = "/usr/include/stdio.h"
 file[8] = "/usr/include/libio.h"
 file[9] = "/usr/include/wchar.h"
 file[10] = "/usr/include/_G_config.h"
 file[11] = "/usr/include/gconv.h"
cuhl = 11, o = 412, asz = 4, osz = 4, ncu = 5752
 file[0] = "???"
 file[1] = "/home/drepper/gnu/new-bu/build/ttt/f.c"
cuhl = 11, o = 0, asz = 4, osz = 4, ncu = 2418
 file[0] = "???"
 file[1] = "/shoggoth/drepper/b.c"
 file[2] = "/home/geoffk/objs/laurel-000912-branch/lib/gcc-lib/powerpc-unknown-linux-gnu/2.96-laurel-000912/include/stddef.h"
 file[3] = "/home/geoffk/objs/laurel-000912-branch/lib/gcc-lib/powerpc-unknown-linux-gnu/2.96-laurel-000912/include/stdarg.h"
 file[4] = "/shoggoth/drepper/<built-in>"
 file[5] = "/usr/include/bits/types.h"
 file[6] = "/usr/include/stdio.h"
 file[7] = "/usr/include/libio.h"
 file[8] = "/usr/include/_G_config.h"
cuhl = 11, o = 213, asz = 4, osz = 4, ncu = 2521
 file[0] = "???"
 file[1] = "/shoggoth/drepper/f.c"
cuhl = 11, o = 267, asz = 4, osz = 4, ncu = 2680
 file[0] = "???"
 file[1] = "/shoggoth/drepper/m.c"
EOF

rm -f testfile testfil2 get-files.out

exit 0

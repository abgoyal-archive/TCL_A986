#! /bin/sh
set -e

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile.bz2 > testfile 2>/dev/null || exit 0

# Don't fail if we cannot decompress the file.
bunzip2 -c $srcdir/testfile2.bz2 > testfile2 2>/dev/null || exit 0

./get-lines testfile testfile2 > get-lines.out

diff -u get-lines.out - <<"EOF"
cuhl = 11, o = 0, asz = 4, osz = 4, ncu = 191
 5 lines
804842c: /home/drepper/gnu/new-bu/build/ttt/m.c:5:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
8048432: /home/drepper/gnu/new-bu/build/ttt/m.c:6:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
804844d: /home/drepper/gnu/new-bu/build/ttt/m.c:7:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
8048458: /home/drepper/gnu/new-bu/build/ttt/m.c:8:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
804845a: /home/drepper/gnu/new-bu/build/ttt/m.c:8:0: is_stmt:yes, end_seq:yes, bb:no, prologue:no, epilogue:no
cuhl = 11, o = 114, asz = 4, osz = 4, ncu = 5617
 4 lines
804845c: /home/drepper/gnu/new-bu/build/ttt/b.c:4:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
804845f: /home/drepper/gnu/new-bu/build/ttt/b.c:5:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
8048464: /home/drepper/gnu/new-bu/build/ttt/b.c:6:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
8048466: /home/drepper/gnu/new-bu/build/ttt/b.c:6:0: is_stmt:yes, end_seq:yes, bb:no, prologue:no, epilogue:no
cuhl = 11, o = 412, asz = 4, osz = 4, ncu = 5752
 4 lines
8048468: /home/drepper/gnu/new-bu/build/ttt/f.c:3:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
804846b: /home/drepper/gnu/new-bu/build/ttt/f.c:4:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
8048470: /home/drepper/gnu/new-bu/build/ttt/f.c:5:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
8048472: /home/drepper/gnu/new-bu/build/ttt/f.c:5:0: is_stmt:yes, end_seq:yes, bb:no, prologue:no, epilogue:no
cuhl = 11, o = 0, asz = 4, osz = 4, ncu = 2418
 4 lines
10000470: /shoggoth/drepper/b.c:4:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
1000047c: /shoggoth/drepper/b.c:5:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
10000480: /shoggoth/drepper/b.c:6:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
10000490: /shoggoth/drepper/b.c:6:0: is_stmt:yes, end_seq:yes, bb:no, prologue:no, epilogue:no
cuhl = 11, o = 213, asz = 4, osz = 4, ncu = 2521
 4 lines
10000490: /shoggoth/drepper/f.c:3:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
1000049c: /shoggoth/drepper/f.c:4:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
100004a0: /shoggoth/drepper/f.c:5:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
100004b0: /shoggoth/drepper/f.c:5:0: is_stmt:yes, end_seq:yes, bb:no, prologue:no, epilogue:no
cuhl = 11, o = 267, asz = 4, osz = 4, ncu = 2680
 5 lines
100004b0: /shoggoth/drepper/m.c:5:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
100004cc: /shoggoth/drepper/m.c:6:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
100004e8: /shoggoth/drepper/m.c:7:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
100004f4: /shoggoth/drepper/m.c:8:0: is_stmt:yes, end_seq:no, bb:no, prologue:no, epilogue:no
10000514: /shoggoth/drepper/m.c:8:0: is_stmt:yes, end_seq:yes, bb:no, prologue:no, epilogue:no
EOF

rm -f testfile testfile2 get-lines.out

exit 0

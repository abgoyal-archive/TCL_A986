#! /bin/sh
lib=../libelf/libelf.a
okfile=arsymtest.ok
tmpfile=arsymtest.tmp
testfile=arsymtest.test

result=0
if test -f $lib; then
    # Generate list using `nm' we check against.
    nm -s $lib |
    sed -e '1,/^Arch/d' -e '/^$/,$d' |
    sort > $okfile

    # Now run our program using libelf.
    ./arsymtest $lib $tmpfile || exit 1
    sort $tmpfile > $testfile
    rm $tmpfile

    # Compare the outputs.
    if cmp $okfile $testfile; then
	result=0
	rm $testfile $okfile
    else
	result=1
    fi
fi

exit $result

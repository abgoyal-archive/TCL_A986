#! /bin/sh
archive=../libelf/.libs/libelf.a
test -f $archive || archive=../libelf/libelf.a
if test -f $archive; then
    # The file is really available (i.e., no shared-only built).
    echo -n "Extracting symbols... $ac_c"

    # The files we are looking at.
    for f in ../libelf/*.o; do
	./arextract $archive `basename $f` arextract.test || exit 1
	cmp $f arextract.test || {
	    echo "Extraction of $1 failed"
	    exit 1
	}
	rm -f ${objpfx}arextract.test
    done

    echo "done"
fi

exit 0

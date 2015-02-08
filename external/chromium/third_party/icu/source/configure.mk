configure:	configure.in ./aclocal.m4
	( autoconf && mv configure configure.tmp && sed -e 's%^ac_cr=.*%ac_cr=`echo X |tr X "\\015"`%'  < configure.tmp > configure && chmod a+rx $@ && rm configure.tmp ) || ( rm $@ ; "echo configure build failed" ; /usr/bin/false  )


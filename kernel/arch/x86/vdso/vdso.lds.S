

#define VDSO_PRELINK 0xffffffffff700000
#include "vdso-layout.lds.S"

VERSION {
	LINUX_2.6 {
	global:
		clock_gettime;
		__vdso_clock_gettime;
		gettimeofday;
		__vdso_gettimeofday;
		getcpu;
		__vdso_getcpu;
	local: *;
	};
}

VDSO64_PRELINK = VDSO_PRELINK;

#define VEXTERN(x)	VDSO64_ ## x = vdso_ ## x;
#include "vextern.h"
#undef	VEXTERN


#ifndef VEXTERN
#include <asm/vsyscall.h>
#define VEXTERN(x) \
	extern typeof(x) *vdso_ ## x __attribute__((visibility("hidden")));
#endif

#define VMAGIC 0xfeedbabeabcdefabUL


VEXTERN(jiffies)
VEXTERN(vgetcpu_mode)
VEXTERN(vsyscall_gtod_data)

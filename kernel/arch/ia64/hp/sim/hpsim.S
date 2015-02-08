
#include <asm/asmmacro.h>

GLOBAL_ENTRY(ia64_ssc)
	mov r15=r36
	break 0x80001
	br.ret.sptk.many rp
END(ia64_ssc)

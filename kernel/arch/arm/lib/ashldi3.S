


#include <linux/linkage.h>

#ifdef __ARMEB__
#define al r1
#define ah r0
#else
#define al r0
#define ah r1
#endif

ENTRY(__ashldi3)
ENTRY(__aeabi_llsl)

	subs	r3, r2, #32
	rsb	ip, r2, #32
	movmi	ah, ah, lsl r2
	movpl	ah, al, lsl r3
 ARM(	orrmi	ah, ah, al, lsr ip	)
 THUMB(	lsrmi	r3, al, ip		)
 THUMB(	orrmi	ah, ah, r3		)
	mov	al, al, lsl r2
	mov	pc, lr

ENDPROC(__ashldi3)
ENDPROC(__aeabi_llsl)

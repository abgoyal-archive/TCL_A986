


#include <linux/linkage.h>

#ifdef __ARMEB__
#define al r1
#define ah r0
#else
#define al r0
#define ah r1
#endif

ENTRY(__ashrdi3)
ENTRY(__aeabi_lasr)

	subs	r3, r2, #32
	rsb	ip, r2, #32
	movmi	al, al, lsr r2
	movpl	al, ah, asr r3
 ARM(	orrmi	al, al, ah, lsl ip	)
 THUMB(	lslmi	r3, ah, ip		)
 THUMB(	orrmi	al, al, r3		)
	mov	ah, ah, asr r2
	mov	pc, lr

ENDPROC(__ashrdi3)
ENDPROC(__aeabi_lasr)

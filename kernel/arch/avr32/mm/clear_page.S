

#include <linux/linkage.h>
#include <asm/page.h>

	.text
	.global clear_page
clear_page:
	sub	r9, r12, -PAGE_SIZE
	mov     r10, 0
	mov	r11, 0
0:      st.d    r12++, r10
	cp      r12, r9
	brne	0b
	mov     pc, lr

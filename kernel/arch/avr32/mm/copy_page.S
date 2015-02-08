
#include <linux/linkage.h>
#include <asm/page.h>

	.text
	.global copy_page
copy_page:
	sub	r10, r11, -(1 << PAGE_SHIFT)
	/* pref	r11[0] */
1:	/* pref	r11[8] */
	ld.d	r8, r11++
	st.d	r12++, r8
	cp	r11, r10
	brlo	1b
	mov	pc, lr



#include <asm/errno.h>


#define csum_partial_copy_nocheck csum_partial_copy_from_user

#include "checksumcopy.S"

	.section .fixup,"ax"

;; Here from the movem loop; restore stack.
4:
	movem	[$sp+],$r8
;; r12 is already decremented.  Add back chunk_size-2.
	addq	40-2,$r12

;; Here from the word loop; r12 is off by 2; add it back.
5:
	addq	2,$r12

;; Here from a failing single byte.
6:

;; Signal in *errptr that we had a failing access.
	moveq	-EFAULT,$r9
	move.d	$r9,[[$sp]]

;; Clear the rest of the destination area using memset.  Preserve the
;; checksum for the readable bytes.
	push	$srp
	push	$r13
	move.d	$r11,$r10
	clear.d	$r11
	jsr	memset
	pop	$r10
	jump	[$sp+]

	.previous
	.section __ex_table,"a"
	.dword 1b,4b
	.dword 2b,5b
	.dword 3b,6b
	.previous

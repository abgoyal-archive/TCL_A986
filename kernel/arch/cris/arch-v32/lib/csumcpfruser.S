

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
	move.d	[$sp],$acr
	moveq	-EFAULT,$r9
	subq	4,$sp
	move.d	$r9,[$acr]

;; Clear the rest of the destination area using memset.  Preserve the
;; checksum for the readable bytes.
	move.d	$r13,[$sp]
	subq	4,$sp
	move.d	$r11,$r10
	move	$srp,[$sp]
	jsr	memset
	clear.d	$r11

	move	[$sp+],$srp
	ret
	move.d	[$sp+],$r10

	.previous
	.section __ex_table,"a"
	.dword 1b,4b
	.dword 2b,5b
	.dword 3b,6b
	.previous

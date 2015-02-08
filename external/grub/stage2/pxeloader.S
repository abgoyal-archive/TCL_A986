
	.file	"pxeloader.S"
	.text

	/* Start with the prehistoric environment... */
	.code16
	
	/* Let's go */
.globl _start; _start:

	/* Jump to the real world */
	ljmp	$0, $0x8200

	/* This region is a junk. Do you say that this is wasteful?
	   But I like that the memory layout of the body is consistent
	   among different stage2s rather than scamping just for 1.5KB. */
	. = _start + 0x8200 - 0x7C00 - 1
	.byte	0

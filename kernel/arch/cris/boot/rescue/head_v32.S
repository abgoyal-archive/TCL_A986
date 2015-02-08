

#include <mach/startup.inc>

#ifdef CONFIG_ETRAX_AXISFLASHMAP

;; Code

	.text
start:

	;; Start clocks for used blocks.
	START_CLOCKS

	move.d	CONFIG_ETRAX_PTABLE_SECTOR, $r10
	jump	$r10			; Jump to decompressor
	nop

#endif

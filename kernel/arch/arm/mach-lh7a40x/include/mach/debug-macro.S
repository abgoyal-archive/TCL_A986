

	@ It is not known if this will be appropriate for every 40x
	@ board.

		.macro  addruart, rx, tmp
		mrc     p15, 0, \rx, c1, c0
		tst     \rx, #1                 @ MMU enabled?
		mov     \rx, #0x00000700        @ offset from base
		orreq   \rx, \rx, #0x80000000   @ physical base
		orrne   \rx, \rx, #0xf8000000   @ virtual base
		.endm

		.macro  senduart,rd,rx
		strb    \rd, [\rx]              @ DATA
		.endm

		.macro  busyuart,rd,rx          @ spin while busy
1001:		ldr     \rd, [\rx, #0x10]       @ STATUS
		tst     \rd, #1 << 3            @ BUSY (TX FIFO not empty)
		bne     1001b                   @ yes, spin
		.endm

		.macro  waituart,rd,rx          @ wait for Tx FIFO room
1001:		ldrb    \rd, [\rx, #0x10]       @ STATUS
		tst     \rd, #1 << 5            @ TXFF (TX FIFO full)
		bne     1001b                   @ yes, spin
		.endm

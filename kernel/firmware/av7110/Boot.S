


	.text
	.align
	.globl _start
_start:
	b reset			// reset vector
	movs  pc, r14           // undefined
	subs  pc, r14, #4       // SWI
	subs  pc, r14, #4       // prefetch abort
	subs  pc, r14, #8       // data abort
	subs  pc, r14, #4       // reserved
	subs  pc, r14, #4       // IRQ
	subs  pc, r14, #4       // FIQ

	.word tbl               // table needed by firmware ROM
tbl:	.word (endtbl - tbl)
	.word 0
  	.word conf
endtbl:	.word 0
conf:	.word 0xa5a55a5a
	.word 0x001f1555
	.word 0x00000009

reset:	ldr	r13, buffer
	ldr	r4, flag
	mov	r0, #0
	str	r0, [r4]
	str	r0, [r4, #4]

	ldr		r1, wait_address
	ldr		r2, flag_address
	ldr		r3, sram

copycode: // copy the code HW Sram
	ldmia	r1!, {r5-r12}
	stmia	r3!, {r5-r12}
	cmp		r1, r2
	ble 	copycode
	ldr		pc, sram // jump to the copied code

wait:	ldrh	r1, [r4]        // wait for flag!=0
 	cmp	r1, #0
	beq	wait

	mov	r1, r13         // buffer address
 	ldr	r3, [r4,#4]     // destaddr

 	ldrh	r2, [r4,#2]     // get segment length
	add     r2, r2, #63     // round length to next 64 bytes
	movs	r2, r2, lsr #6  // and divide by 64
	moveq	r0, #2          // if 0, set flag to 2, else signal
	strh	r0, [r4]        // that buffer is accepted by setting to 0
        beq wait
	
copyloop:
	ldmia	r1!, {r5-r12}
	stmia	r3!, {r5-r12}
	ldmia	r1!, {r5-r12}
	stmia	r3!, {r5-r12}
 	subs	r2, r2, #1
 	bne	copyloop

	eor     r13, r13, #0x1400    // switch to other buffer
	b	wait

// flag is stored at 0x2c0003f8, length at 0x2c0003fa,
// destaddr at 0x2c0003fc

flag:	.word	0x2c0003f8


// buffer 1 is at 0x2c000400, buffer 2 at 0x2c001000

buffer:	.word	0x2c000400

sram: .word   0x9e000800
wait_address: .word   wait
flag_address: .word   flag

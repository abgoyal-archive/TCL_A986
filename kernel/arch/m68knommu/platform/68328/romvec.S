

.global _start
.global _buserr
.global trap
.global system_call

.section .romvec

e_vectors:
.long CONFIG_RAMBASE+CONFIG_RAMSIZE-4, _start, buserr, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
/* TRAP #0-15 */
.long system_call, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long trap, trap, trap, trap
.long 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0


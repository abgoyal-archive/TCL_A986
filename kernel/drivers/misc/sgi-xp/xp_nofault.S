


	.global xp_nofault_PIOR
xp_nofault_PIOR:
	mov	r8=r0			// Stage a success return value
	ld8.acq	r9=[r32];;		// PIO Read the specified register
	adds	r9=1,r9;;		// Add to force consumption
	srlz.i;;			// Allow time for MCA to surface
	br.ret.sptk.many b0;;		// Return success

	.global xp_error_PIOR
xp_error_PIOR:
	mov	r8=1			// Return value of 1
	br.ret.sptk.many b0;;		// Return failure

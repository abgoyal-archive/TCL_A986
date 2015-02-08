


/* PROT ** ICACHE line 1: User DTLB protection trap	*/
	mov		TLB_SFSR, %g1
	stxa		%g0, [%g1] ASI_DMMU		! Clear FaultValid bit
	membar		#Sync				! Synchronize stores
	rdpr		%pstate, %g5			! Move into alt-globals
	wrpr		%g5, PSTATE_AG|PSTATE_MG, %pstate
	rdpr		%tl, %g1			! Need a winfixup?
	cmp		%g1, 1				! Trap level >1?
	mov		TLB_TAG_ACCESS, %g4		! For reload of vaddr

/* PROT ** ICACHE line 2: More real fault processing */
	bgu,pn		%xcc, winfix_trampoline		! Yes, perform winfixup
	 ldxa		[%g4] ASI_DMMU, %g5		! Put tagaccess in %g5
	ba,pt		%xcc, sparc64_realfault_common	! Nope, normal fault
	 mov		FAULT_CODE_DTLB | FAULT_CODE_WRITE, %g4
	nop
	nop
	nop
	nop

/* PROT ** ICACHE line 3: Unused...	*/
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

/* PROT ** ICACHE line 4: Unused...	*/
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

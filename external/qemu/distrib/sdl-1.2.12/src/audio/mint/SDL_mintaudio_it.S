


	.text

	.globl	_SDL_MintAudio_Callback

	.globl	_SDL_MintAudio_XbiosInterrupt
	.globl	_SDL_MintAudio_XbiosInterruptMeasureClock
	.globl	_SDL_MintAudio_Dma8Interrupt
	.globl	_SDL_MintAudio_StfaInterrupt

	.globl	_SDL_MintAudio_mutex
	.globl	_SDL_MintAudio_audiobuf
	.globl	_SDL_MintAudio_numbuf
	.globl	_SDL_MintAudio_audiosize
	.globl	_SDL_MintAudio_clocktics
	.globl	_SDL_MintAudio_hasfpu

	.globl	_SDL_MintAudio_stfa


#define	savptr	0x4a2
#define	savamt	0x46

/*--- Xbios interrupt vector to measure Falcon external clock ---*/

_SDL_MintAudio_XbiosInterruptMeasureClock:          /* 1 mS */

	btst	#0,0xFFFF8901:w	/* state DMA sound */
	beqs	SDL_MintAudio_EndIntMeasure
	addql	#1,_SDL_MintAudio_clocktics
SDL_MintAudio_EndIntMeasure:
	bclr	#5,0xFFFFFA0F:w	/* Clear service bit */
	rte

/*--- Xbios interrupt vector ---*/

_SDL_MintAudio_XbiosInterrupt:

	/* Reenable interrupts, so other interrupts can work */
	movew	#0x2300,sr

	/* Clear service bit, so other MFP interrupts can work */
	bclr	#5,0xfffffa0f:w

	/* Check if we are not already running */
	tstw	_SDL_MintAudio_mutex
	bne	SDL_MintAudio_XbiosEnd
	notw	_SDL_MintAudio_mutex
	
	/* Swap buffers */
	eorw	#1,_SDL_MintAudio_numbuf

	moveml	d0-d7/a0-a6,sp@-

	/* Save FPU if needed */
	tstw	_SDL_MintAudio_hasfpu
	beqs	SDL_MintAudio_Xbios_nofpu1
	.chip	68060
	fsave	sp@-
	fmoveml fpcr/fpsr/fpiar,sp@-
	fmovemx	fp0-fp7,sp@-
	.chip	68000
SDL_MintAudio_Xbios_nofpu1:

	/* Callback */
	jsr	_SDL_MintAudio_Callback

	/* Restore FPU if needed */
	tstw	_SDL_MintAudio_hasfpu
	beqs	SDL_MintAudio_Xbios_nofpu2
	.chip	68060
	fmovemx	sp@+,fp0-fp7
	fmoveml	sp@+,fpcr/fpsr/fpiar
	frestore	sp@+
	.chip	68000
SDL_MintAudio_Xbios_nofpu2:

	/* Reserve space for registers */
	subl	#savamt,savptr

	/* Set new buffer */

	moveq	#0,d0
	movel	_SDL_MintAudio_audiosize,d1

	movew	_SDL_MintAudio_numbuf,d0
	lsll	#2,d0
	lea	_SDL_MintAudio_audiobuf,a0
	movel	a0@(d0:l),a1

	lea	a1@(d1:l),a2

	movel	a2,sp@-
	movel	a1,sp@-
	clrw	sp@-
	movew	#131,sp@-
	trap	#14
	lea	sp@(12),sp

	/* Restore registers space */
	addl	#savamt,savptr

	moveml	sp@+,d0-d7/a0-a6

	clrw	_SDL_MintAudio_mutex
SDL_MintAudio_XbiosEnd:
	rte

/*--- DMA 8 bits interrupt vector ---*/

_SDL_MintAudio_Dma8Interrupt:

	/* Reenable interrupts, so other interrupts can work */
	movew	#0x2300,sr

	/* Clear service bit, so other MFP interrupts can work */
	bclr	#5,0xfffffa0f:w

	/* Check if we are not already running */
	tstw	_SDL_MintAudio_mutex
	bne	SDL_MintAudio_Dma8End
	notw	_SDL_MintAudio_mutex
	
	/* Swap buffers */
	eorw	#1,_SDL_MintAudio_numbuf

	moveml	d0-d1/a0-a1,sp@-

	/* Save FPU if needed */
	tstw	_SDL_MintAudio_hasfpu
	beqs	SDL_MintAudio_Dma8_nofpu1
	.chip	68060
	fsave	sp@-
	fmoveml fpcr/fpsr/fpiar,sp@-
	fmovemx	fp0-fp7,sp@-
	.chip	68000
SDL_MintAudio_Dma8_nofpu1:

	/* Callback */
	jsr	_SDL_MintAudio_Callback

	/* Restore FPU if needed */
	tstw	_SDL_MintAudio_hasfpu
	beqs	SDL_MintAudio_Dma8_nofpu2
	.chip	68060
	fmovemx	sp@+,fp0-fp7
	fmoveml	sp@+,fpcr/fpsr/fpiar
	frestore	sp@+
	.chip	68000
SDL_MintAudio_Dma8_nofpu2:

	/* Set new buffer */

	moveq	#0,d0

	movew	_SDL_MintAudio_numbuf,d0
	lslw	#2,d0
	lea	_SDL_MintAudio_audiobuf,a0
	movel	a0@(d0:w),d1

	/* Modify DMA addresses */
	lea	0xffff8900:w,a0

	moveb	d1,a0@(0x07)	/* Start address */
	rorl	#8,d1
	moveb	d1,a0@(0x05)
	rorl	#8,d1
	moveb	d1,a0@(0x03)
	swap	d1

	addl	_SDL_MintAudio_audiosize,d1

	moveb	d1,a0@(0x13)	/* End address */
	rorl	#8,d1
	moveb	d1,a0@(0x11)
	rorl	#8,d1
	moveb	d1,a0@(0x0f)

	moveml	sp@+,d0-d1/a0-a1

	clrw	_SDL_MintAudio_mutex
SDL_MintAudio_Dma8End:
	rte

/*--- STFA interrupt vector ---*/

STFA_SOUND_START	=	6
STFA_SOUND_END		=	STFA_SOUND_START+8

_SDL_MintAudio_StfaInterrupt:

	/* Reenable interrupts, so other interrupts can work */
	movew	#0x2300,sr

	/* Check if we are not already running */
	tstw	_SDL_MintAudio_mutex
	bnes	SDL_MintAudio_StfaEnd
	notw	_SDL_MintAudio_mutex
	
	/* Swap buffers */
	eorw	#1,_SDL_MintAudio_numbuf

	moveml	d0-d7/a0-a6,sp@-

	/* Save FPU if needed */
	tstw	_SDL_MintAudio_hasfpu
	beqs	SDL_MintAudio_Stfa_nofpu1
	.chip	68060
	fsave	sp@-
	fmoveml fpcr/fpsr/fpiar,sp@-
	fmovemx	fp0-fp7,sp@-
	.chip	68000
SDL_MintAudio_Stfa_nofpu1:

	/* Callback */
	jsr	_SDL_MintAudio_Callback

	/* Restore FPU if needed */
	tstw	_SDL_MintAudio_hasfpu
	beqs	SDL_MintAudio_Stfa_nofpu2
	.chip	68060
	fmovemx	sp@+,fp0-fp7
	fmoveml	sp@+,fpcr/fpsr/fpiar
	frestore	sp@+
	.chip	68000
SDL_MintAudio_Stfa_nofpu2:

	/* Set new buffer */

	moveq	#0,d0
	movel	_SDL_MintAudio_stfa,a1

	movew	_SDL_MintAudio_numbuf,d0
	lslw	#2,d0
	lea	_SDL_MintAudio_audiobuf,a0
	movel	a0@(d0:w),d1

	/* Modify STFA replay buffers */
	movel	d1,a1@(STFA_SOUND_START)
	addl	_SDL_MintAudio_audiosize,d1
	movel	d1,a1@(STFA_SOUND_END)

	moveml	sp@+,d0-d7/a0-a6

	clrw	_SDL_MintAudio_mutex
SDL_MintAudio_StfaEnd:
	rte

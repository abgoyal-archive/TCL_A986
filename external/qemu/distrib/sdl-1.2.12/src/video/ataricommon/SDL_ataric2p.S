


	.globl	_SDL_Atari_C2pConvert
	.globl	_SDL_Atari_C2pConvert8
	.globl	_SDL_Atari_C2pConvert4
	.globl	_SDL_Atari_C2pConvert4_pal

/* ------------	Conversion C2P, 8 bits ------------ */

	.text
_SDL_Atari_C2pConvert8:
	movel	sp@(4),c2p_source
	movel	sp@(8),c2p_dest
	movel	sp@(12),c2p_width
	movel	sp@(16),c2p_height
	movel	sp@(20),c2p_dblligne
	movel	sp@(24),c2p_srcpitch
	movel	sp@(28),c2p_dstpitch

	moveml	d2-d7/a2-a6,sp@-

	movel	c2p_source,c2p_cursrc
	movel	c2p_dest,c2p_curdst
	movel	#0x0f0f0f0f,d4
	movel	#0x00ff00ff,d5
	movel	#0x55555555,d6
	movew	c2p_height+2,c2p_row
	movew	c2p_width+2,d0
	andw	#-8,d0
	movew	d0,c2p_rowlen

SDL_Atari_C2p8_rowloop:

	movel	c2p_cursrc,a0
	movel	c2p_curdst,a1

	movel	a0,a2
	addw	c2p_rowlen,a2

	movel	a0@+,d0
	movel	a0@+,d1
	movel	a0@+,d2
	movel	a0@+,d3
	movel	d1,d7
	lsrl	#4,d7
	eorl	d0,d7
	andl	d4,d7
	eorl	d7,d0
	lsll	#4,d7
	eorl	d7,d1

	movel	d3,d7
	lsrl	#4,d7
	eorl	d2,d7
	andl	d4,d7
	eorl	d7,d2
	lsll	#4,d7
	eorl	d7,d3

	movel	d2,d7
	lsrl	#8,d7
	eorl	d0,d7
 	andl	d5,d7
	eorl	d7,d0
	lsll	#8,d7
	eorl	d7,d2

	movel	d3,d7
	lsrl	#8,d7
	eorl	d1,d7
 	andl	d5,d7
	eorl	d7,d1
	lsll	#8,d7
	eorl	d7,d3
	bras	SDL_Atari_C2p8_start

SDL_Atari_C2p8_pix16:

	movel	a0@+,d0
	movel	a0@+,d1
	movel	a0@+,d2
	movel	a0@+,d3
	movel	d1,d7
	lsrl	#4,d7
	movel	a3,a1@+
	eorl	d0,d7
	andl	d4,d7
	eorl	d7,d0
	lsll	#4,d7
	eorl	d7,d1

	movel	d3,d7
	lsrl	#4,d7
	eorl	d2,d7
	andl	d4,d7
	eorl	d7,d2
	movel	a4,a1@+
	lsll	#4,d7
	eorl	d7,d3

	movel	d2,d7
	lsrl	#8,d7
	eorl	d0,d7
	andl	d5,d7
	eorl	d7,d0
	movel	a5,a1@+
	lsll	#8,d7
	eorl	d7,d2

	movel	d3,d7
	lsrl	#8,d7
	eorl	d1,d7
	andl	d5,d7
	eorl	d7,d1
	movel	a6,a1@+
	lsll	#8,d7
	eorl	d7,d3

SDL_Atari_C2p8_start:

	movel	d2,d7
	lsrl	#1,d7
	eorl	d0,d7
	andl	d6,d7
	eorl	d7,d0
	addl	d7,d7
	eorl	d7,d2

	movel	d3,d7
	lsrl	#1,d7
	eorl	d1,d7
	andl	d6,d7
	eorl	d7,d1
	addl	d7,d7
	eorl	d7,d3
	movew	d2,d7
	movew	d0,d2
	swap	d2
	movew	d2,d0
	movew	d7,d2

	movew	d3,d7
	movew	d1,d3
	swap	d3
	movew	d3,d1
	movew	d7,d3
	movel	d2,d7
	lsrl	#2,d7
	eorl	d0,d7
	andl	#0x33333333,d7
	eorl	d7,d0
	lsll	#2,d7
	eorl	d7,d2

	movel	d3,d7
	lsrl	#2,d7
	eorl	d1,d7
	andl	#0x33333333,d7
	eorl	d7,d1
	lsll	#2,d7
	eorl	d7,d3
	swap	d0
	swap	d1
	swap	d2
	swap	d3

	movel	d0,a6
	movel	d2,a5
	movel	d1,a4
	movel	d3,a3

	cmpl	a0,a2
	bgt		SDL_Atari_C2p8_pix16

	movel	a3,a1@+
	movel	a4,a1@+
	movel	a5,a1@+
	movel	a6,a1@+

	/* Double the line ? */

	movel	c2p_srcpitch,d0
	movel	c2p_dstpitch,d1

	tstl	c2p_dblligne
	beqs	SDL_Atari_C2p8_nodblline

	movel	c2p_curdst,a0
	movel	a0,a1
	addl	d1,a1

	movew	c2p_width+2,d7
	lsrw	#4,d7
	subql	#1,d7
SDL_Atari_C2p8_dblloop:
	movel	a0@+,a1@+
	movel	a0@+,a1@+
	movel	a0@+,a1@+
	movel	a0@+,a1@+
	dbra	d7,SDL_Atari_C2p8_dblloop

	addl	d1,c2p_curdst

SDL_Atari_C2p8_nodblline:

	/* Next line */

	addl	d0,c2p_cursrc
	addl	d1,c2p_curdst

	subqw	#1,c2p_row
	bne		SDL_Atari_C2p8_rowloop	

	moveml	sp@+,d2-d7/a2-a6
	rts

/* ------------	Conversion C2P, 4 bits ------------ */

_SDL_Atari_C2pConvert4:
	movel	sp@(4),c2p_source
	movel	sp@(8),c2p_dest
	movel	sp@(12),c2p_width
	movel	sp@(16),c2p_height
	movel	sp@(20),c2p_dblligne
	movel	sp@(24),c2p_srcpitch
	movel	sp@(28),c2p_dstpitch

	moveml	d2-d7/a2-a6,sp@-

	movel	c2p_source,a0
	movel	c2p_dest,a1
	lea	_SDL_Atari_table_c2p,a2
	movel	#0x00070001,d3
#if defined(__M68020__)
	moveq	#0,d0
#endif
	
	movel	c2p_height,d7
	subql	#1,d7
c2p4_bcly:
	movel	a0,a4	| Save start address of source
	movel	a1,a5	| Save start address of dest

	| Conversion
                 			
	movel	c2p_width,d6
	lsrw	#4,d6
	subql	#1,d6
c2p4_bclx:
	| Octets 0-7
	
	moveq	#0,d1
	moveq	#7,d5
c2p4_bcl07:
#if defined(__M68020__)
	moveb	a0@+,d0
	lea		a2@(0,d0:w:4),a3
#else
	moveq	#0,d0
	moveb	a0@+,d0
	lslw	#2,d0
	lea		a2@(0,d0:w),a3
#endif
	lsll	#1,d1
	orl		a3@,d1
	dbra	d5,c2p4_bcl07

	movepl	d1,a1@(0)
	addw	d3,a1
	swap	d3
	
	| Octets 8-15

	moveq	#0,d1
	moveq	#7,d5
c2p4_bcl815:
#if defined(__M68020__)
	moveb	a0@+,d0
	lea		a2@(0,d0:w:4),a3
#else
	moveq	#0,d0
	moveb	a0@+,d0
	lslw	#2,d0
	lea		a2@(0,d0:w),a3
#endif
	lsll	#1,d1
	orl		a3@,d1
	dbra	d5,c2p4_bcl815

	movepl	d1,a1@(0)
	addw	d3,a1
	swap	d3

	dbra	d6,c2p4_bclx

	| Double line ?

	tstl	c2p_dblligne
	beqs	c2p4_nodblligne

	movel	a5,a6			| src line
	movel	a5,a1			| dest line
	addl	c2p_dstpitch,a1

	movel	c2p_width,d6
	lsrw	#3,d6
	subql	#1,d6
c2p4_copydbl:
	movel	a6@+,a1@+
	dbra	d6,c2p4_copydbl

	addl	c2p_dstpitch,a5
c2p4_nodblligne:

	| Next line

	movel	a4,a0		
	addl	c2p_srcpitch,a0
	movel	a5,a1
	addl	c2p_dstpitch,a1

	dbra	d7,c2p4_bcly

	moveml	sp@+,d2-d7/a2-a6
	rts

/* ------------	Conversion of a light palette in 4 bits ------------ */

_SDL_Atari_C2pConvert4_pal:
	/* a0 is a 256-word light palette */
	movel	sp@(4),a0

	moveml	d2-d3,sp@-

	lea		_SDL_Atari_table_c2p,a1
	movew	#255,d3
c2p_pal_initbcl:
	movew	a0@+,d0
	lsrw	#4,d0
	andw	#15,d0

	moveq	#3,d1
c2p_pal_initbyte:
	btst	d1,d0
	sne		d2
	negw	d2
	moveb	d2,a1@(0,d1:w)

	dbra	d1,c2p_pal_initbyte

	addql	#4,a1
	dbra	d3,c2p_pal_initbcl

	moveml	sp@+,d2-d3

	rts

/* ------------	Buffers ------------ */

	.bss

	.even
	.comm	_SDL_Atari_C2pConvert,4
	.comm	_SDL_Atari_table_c2p,1024

	.comm	c2p_source,4	/* Source framebuffer */
	.comm	c2p_dest,4		/* Destination framebuffer */
	.comm	c2p_width,4		/* Width of zone to convert */
	.comm	c2p_height,4	/* Height of zone to convert */
	.comm	c2p_dblligne,4	/* Double the lines while converting ? */
	.comm	c2p_srcpitch,4	/* Source pitch */
	.comm	c2p_dstpitch,4	/* Destination pitch */
	.comm	c2p_cursrc,4	/* Current source line */
	.comm	c2p_curdst,4	/* Current destination line */
	.comm	c2p_rowlen,2	/* Line length in bytes */
	.comm	c2p_row,2		/* Current line number */

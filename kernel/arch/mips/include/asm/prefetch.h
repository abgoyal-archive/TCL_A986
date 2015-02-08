
#ifndef __ASM_PREFETCH_H
#define __ASM_PREFETCH_H



#define Pref_Load			0
#define Pref_Store			1
						/* 2 and 3 are reserved */
#define Pref_LoadStreamed		4
#define Pref_StoreStreamed		5
#define Pref_LoadRetained		6
#define Pref_StoreRetained		7
						/* 8 ... 24 are reserved */
#define Pref_WriteBackInvalidate	25
#define Pref_PrepareForStore		30

#ifdef __ASSEMBLY__

	.macro	__pref hint addr
#ifdef CONFIG_CPU_HAS_PREFETCH
	pref	\hint, \addr
#endif
	.endm

	.macro	pref_load addr
	__pref	Pref_Load, \addr
	.endm

	.macro	pref_store addr
	__pref	Pref_Store, \addr
	.endm

	.macro	pref_load_streamed addr
	__pref	Pref_LoadStreamed, \addr
	.endm

	.macro	pref_store_streamed addr
	__pref	Pref_StoreStreamed, \addr
	.endm

	.macro	pref_load_retained addr
	__pref	Pref_LoadRetained, \addr
	.endm

	.macro	pref_store_retained addr
	__pref	Pref_StoreRetained, \addr
	.endm

	.macro	pref_wback_inv addr
	__pref	Pref_WriteBackInvalidate, \addr
	.endm

	.macro	pref_prepare_for_store addr
	__pref	Pref_PrepareForStore, \addr
	.endm

#endif

#endif /* __ASM_PREFETCH_H */

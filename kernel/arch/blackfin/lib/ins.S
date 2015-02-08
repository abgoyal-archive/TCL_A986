

#include <linux/linkage.h>
#include <asm/blackfin.h>

.align 2

#ifdef CONFIG_IPIPE
# define DO_CLI \
	[--sp] = rets; \
	[--sp] = (P5:0); \
	sp += -12; \
	call ___ipipe_disable_root_irqs_hw; \
	sp += 12; \
	(P5:0) = [sp++];
# define CLI_INNER_NOP
#else
# define DO_CLI cli R3;
# define CLI_INNER_NOP nop; nop; nop;
#endif

#ifdef CONFIG_IPIPE
# define DO_STI \
	sp += -12; \
	call ___ipipe_enable_root_irqs_hw; \
	sp += 12; \
2:	rets = [sp++];
#else
# define DO_STI 2: sti R3;
#endif

#ifdef CONFIG_BFIN_INS_LOWOVERHEAD
# define CLI_OUTER DO_CLI;
# define STI_OUTER DO_STI;
# define CLI_INNER 1:
# if ANOMALY_05000416
#  define STI_INNER nop; 2: nop;
# else
#  define STI_INNER 2:
# endif
#else
# define CLI_OUTER
# define STI_OUTER
# define CLI_INNER 1: DO_CLI; CLI_INNER_NOP;
# define STI_INNER DO_STI;
#endif


#define COMMON_INS(func, ops) \
ENTRY(_ins##func) \
	P0 = R0;	/* P0 = port */ \
	CLI_OUTER;	/* 3 instructions before first read access */ \
	P1 = R1;	/* P1 = address */ \
	P2 = R2;	/* P2 = count */ \
	SSYNC; \
 \
	LSETUP(1f, 2f) LC0 = P2; \
	CLI_INNER; \
	ops; \
	STI_INNER; \
 \
	STI_OUTER; \
	RTS; \
ENDPROC(_ins##func)

COMMON_INS(l, \
	R0 = [P0]; \
	[P1++] = R0; \
)

COMMON_INS(w, \
	R0 = W[P0]; \
	W[P1++] = R0; \
)

COMMON_INS(w_8, \
	R0 = W[P0]; \
	B[P1++] = R0; \
	R0 = R0 >> 8; \
	B[P1++] = R0; \
)

COMMON_INS(b, \
	R0 = B[P0]; \
	B[P1++] = R0; \
)

COMMON_INS(l_16, \
	R0 = [P0]; \
	W[P1++] = R0; \
	R0 = R0 >> 16; \
	W[P1++] = R0; \
)

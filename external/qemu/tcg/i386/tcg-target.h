
#define TCG_TARGET_I386 1

#define TCG_TARGET_REG_BITS 32
//#define TCG_TARGET_WORDS_BIGENDIAN

#define TCG_TARGET_NB_REGS 8

enum {
    TCG_REG_EAX = 0,
    TCG_REG_ECX,
    TCG_REG_EDX,
    TCG_REG_EBX,
    TCG_REG_ESP,
    TCG_REG_EBP,
    TCG_REG_ESI,
    TCG_REG_EDI,
};

/* used for function call generation */
#define TCG_REG_CALL_STACK TCG_REG_ESP 
#define TCG_TARGET_STACK_ALIGN 16
#define TCG_TARGET_CALL_STACK_OFFSET 0

/* optional instructions */
#define TCG_TARGET_HAS_rot_i32
#define TCG_TARGET_HAS_ext8s_i32
#define TCG_TARGET_HAS_ext16s_i32
#define TCG_TARGET_HAS_ext8u_i32
#define TCG_TARGET_HAS_ext16u_i32
#define TCG_TARGET_HAS_bswap16_i32
#define TCG_TARGET_HAS_bswap32_i32
#define TCG_TARGET_HAS_neg_i32
#define TCG_TARGET_HAS_not_i32
// #define TCG_TARGET_HAS_andc_i32
// #define TCG_TARGET_HAS_orc_i32

#define TCG_TARGET_HAS_GUEST_BASE

/* Note: must be synced with dyngen-exec.h */
#define TCG_AREG0 TCG_REG_EBP
#define TCG_AREG1 TCG_REG_EBX
#define TCG_AREG2 TCG_REG_ESI

static inline void flush_icache_range(unsigned long start, unsigned long stop)
{
}

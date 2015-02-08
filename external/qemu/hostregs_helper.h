


#if defined(DECLARE_HOST_REGS)

#define DO_REG(REG)					\
    register host_reg_t reg_AREG##REG asm(AREG##REG);	\
    volatile host_reg_t saved_AREG##REG;

#elif defined(SAVE_HOST_REGS)

#define DO_REG(REG)					\
    __asm__ __volatile__ ("" : "=r" (reg_AREG##REG));	\
    saved_AREG##REG = reg_AREG##REG;

#else

#define DO_REG(REG)                                     \
    reg_AREG##REG = saved_AREG##REG;		        \
    __asm__ __volatile__ ("" : : "r" (reg_AREG##REG));

#endif

#ifdef AREG0
DO_REG(0)
#endif

#ifdef AREG1
DO_REG(1)
#endif

#ifdef AREG2
DO_REG(2)
#endif

#undef SAVE_HOST_REGS
#undef DECLARE_HOST_REGS
#undef DO_REG

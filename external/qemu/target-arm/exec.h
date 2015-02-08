
#include "config.h"
#include "dyngen-exec.h"

register struct CPUARMState *env asm(AREG0);
register uint32_t T0 asm(AREG1);
register uint32_t T1 asm(AREG2);

#define M0   env->iwmmxt.val

#include "cpu.h"
#include "exec-all.h"

static inline void env_to_regs(void)
{
}

static inline void regs_to_env(void)
{
}

static inline int cpu_has_work(CPUState *env)
{
    return (env->interrupt_request &
            (CPU_INTERRUPT_FIQ | CPU_INTERRUPT_HARD | CPU_INTERRUPT_EXITTB));
}

static inline int cpu_halted(CPUState *env) {
    if (!env->halted)
        return 0;
    /* An interrupt wakes the CPU even if the I and F CPSR bits are
       set.  We use EXITTB to silently wake CPU without causing an
       actual interrupt.  */
    if (cpu_has_work(env)) {
        env->halted = 0;
        return 0;
    }
    return EXCP_HALTED;
}

#if !defined(CONFIG_USER_ONLY)
#include "softmmu_exec.h"
#endif

void raise_exception(int);

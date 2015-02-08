


#include <asm/atomic.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/init_task.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/user.h>
#include <linux/elfcore.h>
#include <linux/mqueue.h>
#include <linux/reboot.h>

//#define DEBUG


static struct signal_struct init_signals = INIT_SIGNALS(init_signals);
static struct sighand_struct init_sighand = INIT_SIGHAND(init_sighand);
union thread_union init_thread_union __init_task_data =
	{ INIT_THREAD_INFO(init_task) };

struct task_struct init_task = INIT_TASK(init_task);

EXPORT_SYMBOL(init_task);


int cris_hlt_counter=0;

void disable_hlt(void)
{
	cris_hlt_counter++;
}

EXPORT_SYMBOL(disable_hlt);

void enable_hlt(void)
{
	cris_hlt_counter--;
}

EXPORT_SYMBOL(enable_hlt);
 
void (*pm_idle)(void);

extern void default_idle(void);

void (*pm_power_off)(void);
EXPORT_SYMBOL(pm_power_off);


void cpu_idle (void)
{
	/* endless idle loop with no priority at all */
	while (1) {
		while (!need_resched()) {
			void (*idle)(void);
			/*
			 * Mark this as an RCU critical section so that
			 * synchronize_kernel() in the unload path waits
			 * for our completion.
			 */
			idle = pm_idle;
			if (!idle)
				idle = default_idle;
			idle();
		}
		preempt_enable_no_resched();
		schedule();
		preempt_disable();
	}
}

void hard_reset_now (void);

void machine_restart(char *cmd)
{
	hard_reset_now();
}


void machine_halt(void)
{
}

/* If or when software power-off is implemented, add code here.  */

void machine_power_off(void)
{
}


void flush_thread(void)
{
}

/* Fill in the fpu structure for a core dump. */
int dump_fpu(struct pt_regs *regs, elf_fpregset_t *fpu)
{
        return 0;
}

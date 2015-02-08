

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/threads.h>

#include <asm/dbell.h>

#ifdef CONFIG_SMP
unsigned long dbell_smp_message[NR_CPUS];

void smp_dbell_message_pass(int target, int msg)
{
	int i;

	if(target < NR_CPUS) {
		set_bit(msg, &dbell_smp_message[target]);
		ppc_msgsnd(PPC_DBELL, 0, target);
	}
	else if(target == MSG_ALL_BUT_SELF) {
		for_each_online_cpu(i) {
			if (i == smp_processor_id())
				continue;
			set_bit(msg, &dbell_smp_message[i]);
			ppc_msgsnd(PPC_DBELL, 0, i);
		}
	}
	else { /* target == MSG_ALL */
		for_each_online_cpu(i)
			set_bit(msg, &dbell_smp_message[i]);
		ppc_msgsnd(PPC_DBELL, PPC_DBELL_MSG_BRDCAST, 0);
	}
}
#endif

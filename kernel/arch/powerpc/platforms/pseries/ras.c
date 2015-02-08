


#include <linux/errno.h>
#include <linux/threads.h>
#include <linux/kernel_stat.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/random.h>
#include <linux/sysrq.h>
#include <linux/bitops.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/cache.h>
#include <asm/prom.h>
#include <asm/ptrace.h>
#include <asm/machdep.h>
#include <asm/rtas.h>
#include <asm/udbg.h>
#include <asm/firmware.h>

#include "pseries.h"

static unsigned char ras_log_buf[RTAS_ERROR_LOG_MAX];
static DEFINE_SPINLOCK(ras_log_buf_lock);

static char mce_data_buf[RTAS_ERROR_LOG_MAX];

static int ras_get_sensor_state_token;
static int ras_check_exception_token;

#define EPOW_SENSOR_TOKEN	9
#define EPOW_SENSOR_INDEX	0
#define RAS_VECTOR_OFFSET	0x500

static irqreturn_t ras_epow_interrupt(int irq, void *dev_id);
static irqreturn_t ras_error_interrupt(int irq, void *dev_id);


static int __init init_ras_IRQ(void)
{
	struct device_node *np;

	ras_get_sensor_state_token = rtas_token("get-sensor-state");
	ras_check_exception_token = rtas_token("check-exception");

	/* Internal Errors */
	np = of_find_node_by_path("/event-sources/internal-errors");
	if (np != NULL) {
		request_event_sources_irqs(np, ras_error_interrupt,
					   "RAS_ERROR");
		of_node_put(np);
	}

	/* EPOW Events */
	np = of_find_node_by_path("/event-sources/epow-events");
	if (np != NULL) {
		request_event_sources_irqs(np, ras_epow_interrupt, "RAS_EPOW");
		of_node_put(np);
	}

	return 0;
}
__initcall(init_ras_IRQ);

static irqreturn_t ras_epow_interrupt(int irq, void *dev_id)
{
	int status = 0xdeadbeef;
	int state = 0;
	int critical;

	status = rtas_call(ras_get_sensor_state_token, 2, 2, &state,
			   EPOW_SENSOR_TOKEN, EPOW_SENSOR_INDEX);

	if (state > 3)
		critical = 1;  /* Time Critical */
	else
		critical = 0;

	spin_lock(&ras_log_buf_lock);

	status = rtas_call(ras_check_exception_token, 6, 1, NULL,
			   RAS_VECTOR_OFFSET,
			   irq_map[irq].hwirq,
			   RTAS_EPOW_WARNING | RTAS_POWERMGM_EVENTS,
			   critical, __pa(&ras_log_buf),
				rtas_get_error_log_max());

	udbg_printf("EPOW <0x%lx 0x%x 0x%x>\n",
		    *((unsigned long *)&ras_log_buf), status, state);
	printk(KERN_WARNING "EPOW <0x%lx 0x%x 0x%x>\n",
	       *((unsigned long *)&ras_log_buf), status, state);

	/* format and print the extended information */
	log_error(ras_log_buf, ERR_TYPE_RTAS_LOG, 0);

	spin_unlock(&ras_log_buf_lock);
	return IRQ_HANDLED;
}

static irqreturn_t ras_error_interrupt(int irq, void *dev_id)
{
	struct rtas_error_log *rtas_elog;
	int status = 0xdeadbeef;
	int fatal;

	spin_lock(&ras_log_buf_lock);

	status = rtas_call(ras_check_exception_token, 6, 1, NULL,
			   RAS_VECTOR_OFFSET,
			   irq_map[irq].hwirq,
			   RTAS_INTERNAL_ERROR, 1 /*Time Critical */,
			   __pa(&ras_log_buf),
				rtas_get_error_log_max());

	rtas_elog = (struct rtas_error_log *)ras_log_buf;

	if ((status == 0) && (rtas_elog->severity >= RTAS_SEVERITY_ERROR_SYNC))
		fatal = 1;
	else
		fatal = 0;

	/* format and print the extended information */
	log_error(ras_log_buf, ERR_TYPE_RTAS_LOG, fatal);

	if (fatal) {
		udbg_printf("Fatal HW Error <0x%lx 0x%x>\n",
			    *((unsigned long *)&ras_log_buf), status);
		printk(KERN_EMERG "Error: Fatal hardware error <0x%lx 0x%x>\n",
		       *((unsigned long *)&ras_log_buf), status);

#ifndef DEBUG_RTAS_POWER_OFF
		/* Don't actually power off when debugging so we can test
		 * without actually failing while injecting errors.
		 * Error data will not be logged to syslog.
		 */
		ppc_md.power_off();
#endif
	} else {
		udbg_printf("Recoverable HW Error <0x%lx 0x%x>\n",
			    *((unsigned long *)&ras_log_buf), status);
		printk(KERN_WARNING
		       "Warning: Recoverable hardware error <0x%lx 0x%x>\n",
		       *((unsigned long *)&ras_log_buf), status);
	}

	spin_unlock(&ras_log_buf_lock);
	return IRQ_HANDLED;
}

static struct rtas_error_log *fwnmi_get_errinfo(struct pt_regs *regs)
{
	unsigned long errdata = regs->gpr[3];
	struct rtas_error_log *errhdr = NULL;
	unsigned long *savep;

	if ((errdata >= 0x7000 && errdata < 0x7fff0) ||
	    (errdata >= rtas.base && errdata < rtas.base + rtas.size - 16)) {
		savep = __va(errdata);
		regs->gpr[3] = savep[0];	/* restore original r3 */
		memset(mce_data_buf, 0, RTAS_ERROR_LOG_MAX);
		memcpy(mce_data_buf, (char *)(savep + 1), RTAS_ERROR_LOG_MAX);
		errhdr = (struct rtas_error_log *)mce_data_buf;
	} else {
		printk("FWNMI: corrupt r3\n");
	}
	return errhdr;
}

static void fwnmi_release_errinfo(void)
{
	int ret = rtas_call(rtas_token("ibm,nmi-interlock"), 0, 1, NULL);
	if (ret != 0)
		printk("FWNMI: nmi-interlock failed: %d\n", ret);
}

int pSeries_system_reset_exception(struct pt_regs *regs)
{
	if (fwnmi_active) {
		struct rtas_error_log *errhdr = fwnmi_get_errinfo(regs);
		if (errhdr) {
			/* XXX Should look at FWNMI information */
		}
		fwnmi_release_errinfo();
	}
	return 0; /* need to perform reset */
}

static int recover_mce(struct pt_regs *regs, struct rtas_error_log * err)
{
	int nonfatal = 0;

	if (err->disposition == RTAS_DISP_FULLY_RECOVERED) {
		/* Platform corrected itself */
		nonfatal = 1;
	} else if ((regs->msr & MSR_RI) &&
		   user_mode(regs) &&
		   err->severity == RTAS_SEVERITY_ERROR_SYNC &&
		   err->disposition == RTAS_DISP_NOT_RECOVERED &&
		   err->target == RTAS_TARGET_MEMORY &&
		   err->type == RTAS_TYPE_ECC_UNCORR &&
		   !(current->pid == 0 || is_global_init(current))) {
		/* Kill off a user process with an ECC error */
		printk(KERN_ERR "MCE: uncorrectable ecc error for pid %d\n",
		       current->pid);
		/* XXX something better for ECC error? */
		_exception(SIGBUS, regs, BUS_ADRERR, regs->nip);
		nonfatal = 1;
	}

	log_error((char *)err, ERR_TYPE_RTAS_LOG, !nonfatal);

	return nonfatal;
}

int pSeries_machine_check_exception(struct pt_regs *regs)
{
	struct rtas_error_log *errp;

	if (fwnmi_active) {
		errp = fwnmi_get_errinfo(regs);
		fwnmi_release_errinfo();
		if (errp && recover_mce(regs, errp))
			return 1;
	}

	return 0;
}

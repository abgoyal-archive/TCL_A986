


/* This file contains some function for controlling the access to the  */
/* ST-DMA chip that may be shared between devices. Currently we have:  */
/*   TT:     Floppy and ACSI bus                                       */
/*   Falcon: Floppy and SCSI                                           */
/*                                                                     */
/* The controlling functions set up a wait queue for access to the     */
/* ST-DMA chip. Callers to stdma_lock() that cannot granted access are */
/* put onto a queue and waked up later if the owner calls              */
/* stdma_release(). Additionally, the caller gives his interrupt       */
/* service routine to stdma_lock().                                    */
/*                                                                     */
/* On the Falcon, the IDE bus uses just the ACSI/Floppy interrupt, but */
/* not the ST-DMA chip itself. So falhd.c needs not to lock the        */
/* chip. The interrupt is routed to falhd.c if IDE is configured, the  */
/* model is a Falcon and the interrupt was caused by the HD controller */
/* (can be determined by looking at its status register).              */


#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/genhd.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/module.h>

#include <asm/atari_stdma.h>
#include <asm/atariints.h>
#include <asm/atarihw.h>
#include <asm/io.h>
#include <asm/irq.h>

static int stdma_locked;			/* the semaphore */
						/* int func to be called */
static irq_handler_t stdma_isr;
static void *stdma_isr_data;			/* data passed to isr */
static DECLARE_WAIT_QUEUE_HEAD(stdma_wait);	/* wait queue for ST-DMA */




/***************************** Prototypes *****************************/

static irqreturn_t stdma_int (int irq, void *dummy);

/************************* End of Prototypes **************************/




void stdma_lock(irq_handler_t handler, void *data)
{
	unsigned long flags;

	local_irq_save(flags);		/* protect lock */

	/* Since the DMA is used for file system purposes, we
	 have to sleep uninterruptible (there may be locked
	 buffers) */
	wait_event(stdma_wait, !stdma_locked);

	stdma_locked   = 1;
	stdma_isr      = handler;
	stdma_isr_data = data;
	local_irq_restore(flags);
}
EXPORT_SYMBOL(stdma_lock);



void stdma_release(void)
{
	unsigned long flags;

	local_irq_save(flags);

	stdma_locked   = 0;
	stdma_isr      = NULL;
	stdma_isr_data = NULL;
	wake_up(&stdma_wait);

	local_irq_restore(flags);
}
EXPORT_SYMBOL(stdma_release);



int stdma_others_waiting(void)
{
	return waitqueue_active(&stdma_wait);
}
EXPORT_SYMBOL(stdma_others_waiting);



int stdma_islocked(void)
{
	return stdma_locked;
}
EXPORT_SYMBOL(stdma_islocked);



void __init stdma_init(void)
{
	stdma_isr = NULL;
	if (request_irq(IRQ_MFP_FDC, stdma_int, IRQ_TYPE_SLOW | IRQF_SHARED,
			"ST-DMA: floppy/ACSI/IDE/Falcon-SCSI", stdma_int))
		pr_err("Couldn't register ST-DMA interrupt\n");
}



static irqreturn_t stdma_int(int irq, void *dummy)
{
  if (stdma_isr)
      (*stdma_isr)(irq, stdma_isr_data);
  return IRQ_HANDLED;
}

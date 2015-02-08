

#include <asm/prom.h>

#include "pseries.h"

void request_event_sources_irqs(struct device_node *np,
				irq_handler_t handler,
				const char *name)
{
	int i, index, count = 0;
	struct of_irq oirq;
	const u32 *opicprop;
	unsigned int opicplen;
	unsigned int virqs[16];

	/* Check for obsolete "open-pic-interrupt" property. If present, then
	 * map those interrupts using the default interrupt host and default
	 * trigger
	 */
	opicprop = of_get_property(np, "open-pic-interrupt", &opicplen);
	if (opicprop) {
		opicplen /= sizeof(u32);
		for (i = 0; i < opicplen; i++) {
			if (count > 15)
				break;
			virqs[count] = irq_create_mapping(NULL, *(opicprop++));
			if (virqs[count] == NO_IRQ)
				printk(KERN_ERR "Unable to allocate interrupt "
				       "number for %s\n", np->full_name);
			else
				count++;

		}
	}
	/* Else use normal interrupt tree parsing */
	else {
		/* First try to do a proper OF tree parsing */
		for (index = 0; of_irq_map_one(np, index, &oirq) == 0;
		     index++) {
			if (count > 15)
				break;
			virqs[count] = irq_create_of_mapping(oirq.controller,
							    oirq.specifier,
							    oirq.size);
			if (virqs[count] == NO_IRQ)
				printk(KERN_ERR "Unable to allocate interrupt "
				       "number for %s\n", np->full_name);
			else
				count++;
		}
	}

	/* Now request them */
	for (i = 0; i < count; i++) {
		if (request_irq(virqs[i], handler, 0, name, NULL)) {
			printk(KERN_ERR "Unable to request interrupt %d for "
			       "%s\n", virqs[i], np->full_name);
			return;
		}
	}
}


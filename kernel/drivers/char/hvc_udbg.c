

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/irq.h>

#include <asm/udbg.h>

#include "hvc_console.h"

struct hvc_struct *hvc_udbg_dev;

static int hvc_udbg_put(uint32_t vtermno, const char *buf, int count)
{
	int i;

	for (i = 0; i < count; i++)
		udbg_putc(buf[i]);

	return i;
}

static int hvc_udbg_get(uint32_t vtermno, char *buf, int count)
{
	int i, c;

	if (!udbg_getc_poll)
		return 0;

	for (i = 0; i < count; i++) {
		if ((c = udbg_getc_poll()) == -1)
			break;
		buf[i] = c;
	}

	return i;
}

static const struct hv_ops hvc_udbg_ops = {
	.get_chars = hvc_udbg_get,
	.put_chars = hvc_udbg_put,
};

static int __init hvc_udbg_init(void)
{
	struct hvc_struct *hp;

	BUG_ON(hvc_udbg_dev);

	hp = hvc_alloc(0, NO_IRQ, &hvc_udbg_ops, 16);
	if (IS_ERR(hp))
		return PTR_ERR(hp);

	hvc_udbg_dev = hp;

	return 0;
}
module_init(hvc_udbg_init);

static void __exit hvc_udbg_exit(void)
{
	if (hvc_udbg_dev)
		hvc_remove(hvc_udbg_dev);
}
module_exit(hvc_udbg_exit);

static int __init hvc_udbg_console_init(void)
{
	hvc_instantiate(0, 0, &hvc_udbg_ops);
	add_preferred_console("hvc", 0, NULL);

	return 0;
}
console_initcall(hvc_udbg_console_init);

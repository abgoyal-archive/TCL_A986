

#include <linux/stddef.h>
#include <asm/errno.h>
#include <linux/sysrq.h>
#include <linux/ctype.h>

#include "ctrlchar.h"

#ifdef CONFIG_MAGIC_SYSRQ
static int ctrlchar_sysrq_key;
static struct tty_struct *sysrq_tty;

static void
ctrlchar_handle_sysrq(struct work_struct *work)
{
	handle_sysrq(ctrlchar_sysrq_key, sysrq_tty);
}

static DECLARE_WORK(ctrlchar_work, ctrlchar_handle_sysrq);
#endif


unsigned int
ctrlchar_handle(const unsigned char *buf, int len, struct tty_struct *tty)
{
	if ((len < 2) || (len > 3))
		return CTRLCHAR_NONE;

	/* hat is 0xb1 in codepage 037 (US etc.) and thus */
	/* converted to 0x5e in ascii ('^') */
	if ((buf[0] != '^') && (buf[0] != '\252'))
		return CTRLCHAR_NONE;

#ifdef CONFIG_MAGIC_SYSRQ
	/* racy */
	if (len == 3 && buf[1] == '-') {
		ctrlchar_sysrq_key = buf[2];
		sysrq_tty = tty;
		schedule_work(&ctrlchar_work);
		return CTRLCHAR_SYSRQ;
	}
#endif

	if (len != 2)
		return CTRLCHAR_NONE;

	switch (tolower(buf[1])) {
	case 'c':
		return INTR_CHAR(tty) | CTRLCHAR_CTRL;
	case 'd':
		return EOF_CHAR(tty)  | CTRLCHAR_CTRL;
	case 'z':
		return SUSP_CHAR(tty) | CTRLCHAR_CTRL;
	}
	return CTRLCHAR_NONE;
}



#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/openprom.h>
#include <asm/oplib.h>
#include <asm/system.h>
#include <linux/string.h>

extern void restore_current(void);

int
prom_nbgetchar(void)
{
	static char inc;
	int i = -1;
	unsigned long flags;

	spin_lock_irqsave(&prom_lock, flags);
	switch(prom_vers) {
	case PROM_V0:
		i = (*(romvec->pv_nbgetchar))();
		break;
	case PROM_V2:
	case PROM_V3:
		if( (*(romvec->pv_v2devops).v2_dev_read)(*romvec->pv_v2bootargs.fd_stdin , &inc, 0x1) == 1) {
			i = inc;
		} else {
			i = -1;
		}
		break;
	default:
		i = -1;
		break;
	};
	restore_current();
	spin_unlock_irqrestore(&prom_lock, flags);
	return i; /* Ugh, we could spin forever on unsupported proms ;( */
}

int
prom_nbputchar(char c)
{
	static char outc;
	unsigned long flags;
	int i = -1;

	spin_lock_irqsave(&prom_lock, flags);
	switch(prom_vers) {
	case PROM_V0:
		i = (*(romvec->pv_nbputchar))(c);
		break;
	case PROM_V2:
	case PROM_V3:
		outc = c;
		if( (*(romvec->pv_v2devops).v2_dev_write)(*romvec->pv_v2bootargs.fd_stdout, &outc, 0x1) == 1)
			i = 0;
		else
			i = -1;
		break;
	default:
		i = -1;
		break;
	};
	restore_current();
	spin_unlock_irqrestore(&prom_lock, flags);
	return i; /* Ugh, we could spin forever on unsupported proms ;( */
}

/* Blocking version of get character routine above. */
char
prom_getchar(void)
{
	int character;
	while((character = prom_nbgetchar()) == -1) ;
	return (char) character;
}

/* Blocking version of put character routine above. */
void
prom_putchar(char c)
{
	while(prom_nbputchar(c) == -1) ;
}

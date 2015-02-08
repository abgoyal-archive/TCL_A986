

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/openprom.h>
#include <asm/oplib.h>
#include <asm/system.h>
#include <linux/string.h>

extern int prom_stdin, prom_stdout;

inline int
prom_nbgetchar(void)
{
	unsigned long args[7];
	char inc;

	args[0] = (unsigned long) "read";
	args[1] = 3;
	args[2] = 1;
	args[3] = (unsigned int) prom_stdin;
	args[4] = (unsigned long) &inc;
	args[5] = 1;
	args[6] = (unsigned long) -1;

	p1275_cmd_direct(args);

	if (args[6] == 1)
		return inc;
	return -1;
}

inline int
prom_nbputchar(char c)
{
	unsigned long args[7];
	char outc;
	
	outc = c;

	args[0] = (unsigned long) "write";
	args[1] = 3;
	args[2] = 1;
	args[3] = (unsigned int) prom_stdout;
	args[4] = (unsigned long) &outc;
	args[5] = 1;
	args[6] = (unsigned long) -1;

	p1275_cmd_direct(args);

	if (args[6] == 1)
		return 0;
	else
		return -1;
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
	prom_nbputchar(c);
}

void
prom_puts(const char *s, int len)
{
	unsigned long args[7];

	args[0] = (unsigned long) "write";
	args[1] = 3;
	args[2] = 1;
	args[3] = (unsigned int) prom_stdout;
	args[4] = (unsigned long) s;
	args[5] = len;
	args[6] = (unsigned long) -1;

	p1275_cmd_direct(args);
}

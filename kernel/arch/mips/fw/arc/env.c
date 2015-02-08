
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>

#include <asm/fw/arc/types.h>
#include <asm/sgialib.h>

PCHAR __init
ArcGetEnvironmentVariable(CHAR *name)
{
	return (CHAR *) ARC_CALL1(get_evar, name);
}

LONG __init
ArcSetEnvironmentVariable(PCHAR name, PCHAR value)
{
	return ARC_CALL2(set_evar, name, value);
}

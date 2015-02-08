

#include <linux/module.h>
#include <linux/init.h>
#include <linux/string.h>

#include <asm/bootinfo.h>

int prom_argc;
char **prom_argv;
char **prom_envp;

void prom_init_cmdline(void)
{
	int i;

	for (i = 1; i < prom_argc; i++) {
		strlcat(arcs_cmdline, prom_argv[i], COMMAND_LINE_SIZE);
		if (i < (prom_argc - 1))
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
	}
}

char *prom_getenv(char *envname)
{
	/*
	 * Return a pointer to the given environment variable.
	 * YAMON uses "name", "value" pairs, while U-Boot uses "name=value".
	 */

	char **env = prom_envp;
	int i = strlen(envname);
	int yamon = (*env && strchr(*env, '=') == NULL);

	while (*env) {
		if (yamon) {
			if (strcmp(envname, *env++) == 0)
				return *env;
		} else if (strncmp(envname, *env, i) == 0 && (*env)[i] == '=')
			return *env + i + 1;
		env++;
	}

	return NULL;
}

static inline unsigned char str2hexnum(unsigned char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return 0; /* foo */
}

static inline void str2eaddr(unsigned char *ea, unsigned char *str)
{
	int i;

	for (i = 0; i < 6; i++) {
		unsigned char num;

		if ((*str == '.') || (*str == ':'))
			str++;
		num  = str2hexnum(*str++) << 4;
		num |= str2hexnum(*str++);
		ea[i] = num;
	}
}

int prom_get_ethernet_addr(char *ethernet_addr)
{
	char *ethaddr_str;

	/* Check the environment variables first */
	ethaddr_str = prom_getenv("ethaddr");
	if (!ethaddr_str) {
		/* Check command line */
		ethaddr_str = strstr(arcs_cmdline, "ethaddr=");
		if (!ethaddr_str)
			return -1;

		ethaddr_str += strlen("ethaddr=");
	}

	str2eaddr(ethernet_addr, ethaddr_str);

	return 0;
}
EXPORT_SYMBOL(prom_get_ethernet_addr);

void __init prom_free_prom_memory(void)
{
}

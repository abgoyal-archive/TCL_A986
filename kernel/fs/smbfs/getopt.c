

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/net.h>

#include "getopt.h"

int smb_getopt(char *caller, char **options, struct option *opts,
	       char **optopt, char **optarg, unsigned long *flag,
	       unsigned long *value)
{
	char *token;
	char *val;
	int i;

	do {
		if ((token = strsep(options, ",")) == NULL)
			return 0;
	} while (*token == '\0');
	*optopt = token;

	*optarg = NULL;
	if ((val = strchr (token, '=')) != NULL) {
		*val++ = 0;
		if (value)
			*value = simple_strtoul(val, NULL, 0);
		*optarg = val;
	}

	for (i = 0; opts[i].name != NULL; i++) {
		if (!strcmp(opts[i].name, token)) {
			if (!opts[i].flag && (!val || !*val)) {
				printk("%s: the %s option requires an argument\n",
				       caller, token);
				return -1;
			}

			if (flag && opts[i].flag)
				*flag |= opts[i].flag;

			return opts[i].val;
		}
	}
	printk("%s: Unrecognized mount option %s\n", caller, token);
	return -1;
}

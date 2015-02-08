
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "kword.h"
#include "parser.h"

int lineno;

struct keyword_t rfcomm_keyword[] = {
	{ "bind",	K_BIND		},
	{ "device",	K_DEVICE	},
	{ "channel",	K_CHANNEL	},
	{ "comment",	K_COMMENT	},

	{ "yes",	K_YES		},
	{ "no",		K_NO		},
	{ "enable",	K_YES		},
	{ "disable",	K_NO		},

	{ NULL , 0 }
};

int rfcomm_find_keyword(struct keyword_t *keyword, char *string)
{
	while (keyword->string) {
		if (!strcmp(string, keyword->string))
			return keyword->type;
		keyword++;
	}

	return -1;
}

struct rfcomm_opts rfcomm_opts[RFCOMM_MAX_DEV];

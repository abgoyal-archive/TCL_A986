

#ifndef _LINUX_KEYRESET_H
#define _LINUX_KEYRESET_H

#define KEYRESET_NAME "keyreset"

struct keyreset_platform_data {
	int *keys_up;
	int keys_down[]; /* 0 terminated */
};

#endif /* _LINUX_KEYRESET_H */

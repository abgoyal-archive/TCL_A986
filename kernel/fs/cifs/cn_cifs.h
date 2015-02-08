

#ifndef _CN_CIFS_H
#define _CN_CIFS_H
#ifdef CONFIG_CIFS_UPCALL
#include <linux/types.h>
#include <linux/connector.h>

struct cifs_upcall {
	char signature[4]; /* CIFS */
	enum command {
		CIFS_GET_IP = 0x00000001,   /* get ip address for hostname */
		CIFS_GET_SECBLOB = 0x00000002, /* get SPNEGO wrapped blob */
	} command;
	/* union cifs upcall data follows */
};
#endif /* CIFS_UPCALL */
#endif /* _CN_CIFS_H */

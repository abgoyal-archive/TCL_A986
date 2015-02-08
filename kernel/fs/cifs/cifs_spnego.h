

#ifndef _CIFS_SPNEGO_H
#define _CIFS_SPNEGO_H

#define CIFS_SPNEGO_UPCALL_VERSION 2

struct cifs_spnego_msg {
	uint32_t	version;
	uint32_t	flags;
	uint32_t	sesskey_len;
	uint32_t	secblob_len;
	uint8_t		data[1];
};

#ifdef __KERNEL__
extern struct key_type cifs_spnego_key_type;
extern struct key *cifs_get_spnego_key(struct cifsSesInfo *sesInfo);
#endif /* KERNEL */

#endif /* _CIFS_SPNEGO_H */

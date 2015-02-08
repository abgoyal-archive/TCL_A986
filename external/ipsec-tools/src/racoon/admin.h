/*	$NetBSD: admin.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: admin.h,v 1.11 2005/06/19 22:37:47 manubsd Exp */


#ifndef _ADMIN_H
#define _ADMIN_H

#define ADMINSOCK_PATH ADMINPORTDIR "/racoon.sock"

extern char *adminsock_path;
extern uid_t adminsock_owner;
extern gid_t adminsock_group;
extern mode_t adminsock_mode;

/* command for administration. */
/* NOTE: host byte order. */
struct admin_com {
	u_int16_t ac_len;	/* total packet length including data */
	u_int16_t ac_cmd;
	int16_t ac_errno;
	u_int16_t ac_proto;
};

#define ADMIN_RELOAD_CONF	0x0001
#define ADMIN_SHOW_SCHED	0x0002
#define ADMIN_SHOW_EVT		0x0003

#define ADMIN_SHOW_SA		0x0101
#define ADMIN_FLUSH_SA		0x0102

#define ADMIN_DELETE_SA		0x0201
#define ADMIN_ESTABLISH_SA	0x0202
#define ADMIN_DELETE_ALL_SA_DST	0x0204	/* All SA for a given peer */

#define ADMIN_ESTABLISH_SA_PSK	0x0203

#define ADMIN_LOGOUT_USER	0x0205  /* Delete SA for a given Xauth user */


/* the value of proto */
#define ADMIN_PROTO_ISAKMP	0x01ff
#define ADMIN_PROTO_IPSEC	0x02ff
#define ADMIN_PROTO_AH		0x0201
#define ADMIN_PROTO_ESP		0x0202
#define ADMIN_PROTO_INTERNAL	0x0301

struct admin_com_indexes {
	u_int8_t prefs;
	u_int8_t prefd;
	u_int8_t ul_proto;
	u_int8_t reserved;
	struct sockaddr_storage src;
	struct sockaddr_storage dst;
};

struct admin_com_psk { 
	int id_type;
	size_t id_len;
	size_t key_len;
	/* Followed by id and key */
}; 

extern int admin2pfkey_proto __P((u_int));

#endif /* _ADMIN_H */

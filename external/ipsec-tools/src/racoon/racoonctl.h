/*	$NetBSD: racoonctl.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: racoonctl.h,v 1.3 2005/06/19 22:37:47 manubsd Exp */


#ifndef _RACOONCTL_H
#define _RACOONCTL_H

/* bumped on any change to the interface */
#define RACOONCTL_INTERFACE	20050619
extern u_int32_t racoonctl_interface;

/* bumped when introducing changes that break backward compatibility */
#define RACOONCTL_INTERFACE_MAJOR	1	
extern u_int32_t racoonctl_interface_major;

extern u_int32_t loglevel;

int com_init(void);
int com_send(vchar_t *);
int com_recv(vchar_t **);
struct sockaddr *get_sockaddr(int, char *, char *);

#endif /* _RACOONCTL_H */


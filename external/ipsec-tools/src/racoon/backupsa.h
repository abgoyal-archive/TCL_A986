/*	$NetBSD: backupsa.h,v 1.5 2006/12/09 05:52:57 manu Exp $	*/

/* Id: backupsa.h,v 1.3 2004/06/11 16:00:15 ludvigm Exp */


#ifndef _BACKUPSA_H
#define _BACKUPSA_H

extern int backupsa_to_file __P((struct pfkey_send_sa_args *));
extern int backupsa_from_file __P((void));
extern int backupsa_clean __P((void));

#endif /* _BACKUPSA_H */

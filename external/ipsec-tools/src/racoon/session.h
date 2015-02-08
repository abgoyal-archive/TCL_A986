/*	$NetBSD: session.h,v 1.4 2006/09/09 16:22:10 manu Exp $	*/

/* Id: session.h,v 1.3 2004/06/11 16:00:17 ludvigm Exp */


#ifndef _SESSION_H
#define _SESSION_H

extern int session __P((void));
extern RETSIGTYPE signal_handler __P((int));

#endif /* _SESSION_H */

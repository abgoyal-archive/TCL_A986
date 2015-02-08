/*	$NetBSD: genlist.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/* Id: genlist.h,v 1.2 2004/07/12 20:43:50 ludvigm Exp */


#ifndef _GENLIST_H
#define _GENLIST_H

#include <sys/queue.h>

/* See the bottom of genlist.c for example use. */

/* This declares 'struct genlist' */
TAILQ_HEAD(genlist, genlist_entry);

/* This is where the data are actually stored. */
struct genlist_entry {
	void *data;
	TAILQ_ENTRY(genlist_entry) chain;
};

/* This function returns an initialized list head. */
struct genlist *genlist_init (void);

/* Insert an entry at the beginning/end og the list. */
struct genlist_entry *genlist_insert (struct genlist *head, void *data);
struct genlist_entry *genlist_append (struct genlist *head, void *data);

typedef void *(genlist_func_t)(void *entry, void *arg);

void *genlist_foreach (struct genlist *head, genlist_func_t func, void *arg);

void *genlist_next (struct genlist *head, struct genlist_entry **buf);

typedef void (genlist_freedata_t)(void *entry);

void genlist_free (struct genlist *head, genlist_freedata_t func);

#endif /* _GENLIST_H */

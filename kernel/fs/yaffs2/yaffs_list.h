


#ifndef __YAFFS_LIST_H__
#define __YAFFS_LIST_H__


#include "yportenv.h"


struct ylist_head {
	struct ylist_head *next; /* next in chain */
	struct ylist_head *prev; /* previous in chain */
};


/* Initialise a static list */
#define YLIST_HEAD(name) \
struct ylist_head name = { &(name), &(name)}



/* Initialise a list head to an empty list */
#define YINIT_LIST_HEAD(p) \
do { \
	(p)->next = (p);\
	(p)->prev = (p); \
} while (0)


/* Add an element to a list */
static Y_INLINE void ylist_add(struct ylist_head *newEntry,
				struct ylist_head *list)
{
	struct ylist_head *listNext = list->next;

	list->next = newEntry;
	newEntry->prev = list;
	newEntry->next = listNext;
	listNext->prev = newEntry;

}

static Y_INLINE void ylist_add_tail(struct ylist_head *newEntry,
				 struct ylist_head *list)
{
	struct ylist_head *listPrev = list->prev;

	list->prev = newEntry;
	newEntry->next = list;
	newEntry->prev = listPrev;
	listPrev->next = newEntry;

}


static Y_INLINE void ylist_del(struct ylist_head *entry)
{
	struct ylist_head *listNext = entry->next;
	struct ylist_head *listPrev = entry->prev;

	listNext->prev = listPrev;
	listPrev->next = listNext;

}

static Y_INLINE void ylist_del_init(struct ylist_head *entry)
{
	ylist_del(entry);
	entry->next = entry->prev = entry;
}


/* Test if the list is empty */
static Y_INLINE int ylist_empty(struct ylist_head *entry)
{
	return (entry->next == entry);
}




#define ylist_entry(entry, type, member) \
	((type *)((char *)(entry)-(unsigned long)(&((type *)NULL)->member)))



#define ylist_for_each(itervar, list) \
	for (itervar = (list)->next; itervar != (list); itervar = itervar->next)

#define ylist_for_each_safe(itervar, saveVar, list) \
	for (itervar = (list)->next, saveVar = (list)->next->next; \
		itervar != (list); itervar = saveVar, saveVar = saveVar->next)


#endif

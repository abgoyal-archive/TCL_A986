
#ifndef _H_JFS_UNICODE
#define _H_JFS_UNICODE

#include <linux/slab.h>
#include <asm/byteorder.h>
#include "jfs_types.h"

typedef struct {
	wchar_t start;
	wchar_t end;
	signed char *table;
} UNICASERANGE;

extern signed char UniUpperTable[512];
extern UNICASERANGE UniUpperRange[];
extern int get_UCSname(struct component_name *, struct dentry *);
extern int jfs_strfromUCS_le(char *, const __le16 *, int, struct nls_table *);

#define free_UCSname(COMP) kfree((COMP)->name)

static inline wchar_t *UniStrcpy(wchar_t * ucs1, const wchar_t * ucs2)
{
	wchar_t *anchor = ucs1;	/* save the start of result string */

	while ((*ucs1++ = *ucs2++));
	return anchor;
}



static inline __le16 *UniStrncpy_le(__le16 * ucs1, const __le16 * ucs2,
				  size_t n)
{
	__le16 *anchor = ucs1;

	while (n-- && *ucs2)	/* Copy the strings */
		*ucs1++ = *ucs2++;

	n++;
	while (n--)		/* Pad with nulls */
		*ucs1++ = 0;
	return anchor;
}

static inline int UniStrncmp_le(const wchar_t * ucs1, const __le16 * ucs2,
				size_t n)
{
	if (!n)
		return 0;	/* Null strings are equal */
	while ((*ucs1 == __le16_to_cpu(*ucs2)) && *ucs1 && --n) {
		ucs1++;
		ucs2++;
	}
	return (int) *ucs1 - (int) __le16_to_cpu(*ucs2);
}

static inline __le16 *UniStrncpy_to_le(__le16 * ucs1, const wchar_t * ucs2,
				       size_t n)
{
	__le16 *anchor = ucs1;

	while (n-- && *ucs2)	/* Copy the strings */
		*ucs1++ = cpu_to_le16(*ucs2++);

	n++;
	while (n--)		/* Pad with nulls */
		*ucs1++ = 0;
	return anchor;
}

static inline wchar_t *UniStrncpy_from_le(wchar_t * ucs1, const __le16 * ucs2,
					  size_t n)
{
	wchar_t *anchor = ucs1;

	while (n-- && *ucs2)	/* Copy the strings */
		*ucs1++ = __le16_to_cpu(*ucs2++);

	n++;
	while (n--)		/* Pad with nulls */
		*ucs1++ = 0;
	return anchor;
}

static inline wchar_t UniToupper(wchar_t uc)
{
	UNICASERANGE *rp;

	if (uc < sizeof(UniUpperTable)) {	/* Latin characters */
		return uc + UniUpperTable[uc];	/* Use base tables */
	} else {
		rp = UniUpperRange;	/* Use range tables */
		while (rp->start) {
			if (uc < rp->start)	/* Before start of range */
				return uc;	/* Uppercase = input */
			if (uc <= rp->end)	/* In range */
				return uc + rp->table[uc - rp->start];
			rp++;	/* Try next range */
		}
	}
	return uc;		/* Past last range */
}


static inline wchar_t *UniStrupr(wchar_t * upin)
{
	wchar_t *up;

	up = upin;
	while (*up) {		/* For all characters */
		*up = UniToupper(*up);
		up++;
	}
	return upin;		/* Return input pointer */
}

#endif				/* !_H_JFS_UNICODE */



#include <asm/byteorder.h>
#include <linux/types.h>
#include <linux/nls.h>

#define  UNIUPR_NOLOWER		/* Example to not expand lower case tables */

#define UNI_ASTERIK     (__u16) ('*' + 0xF000)
#define UNI_QUESTION    (__u16) ('?' + 0xF000)
#define UNI_COLON       (__u16) (':' + 0xF000)
#define UNI_GRTRTHAN    (__u16) ('>' + 0xF000)
#define UNI_LESSTHAN    (__u16) ('<' + 0xF000)
#define UNI_PIPE        (__u16) ('|' + 0xF000)
#define UNI_SLASH       (__u16) ('\\' + 0xF000)

#ifndef	UNICASERANGE_DEFINED
struct UniCaseRange {
	wchar_t start;
	wchar_t end;
	signed char *table;
};
#endif				/* UNICASERANGE_DEFINED */

#ifndef UNIUPR_NOUPPER
extern signed char CifsUniUpperTable[512];
extern const struct UniCaseRange CifsUniUpperRange[];
#endif				/* UNIUPR_NOUPPER */

#ifndef UNIUPR_NOLOWER
extern signed char UniLowerTable[512];
extern struct UniCaseRange UniLowerRange[];
#endif				/* UNIUPR_NOLOWER */

#ifdef __KERNEL__
int cifs_from_ucs2(char *to, const __le16 *from, int tolen, int fromlen,
		   const struct nls_table *codepage, bool mapchar);
int cifs_ucs2_bytes(const __le16 *from, int maxbytes,
		    const struct nls_table *codepage);
int cifs_strtoUCS(__le16 *, const char *, int, const struct nls_table *);
char *cifs_strndup_from_ucs(const char *src, const int maxlen,
			    const bool is_unicode,
			    const struct nls_table *codepage);
#endif

static inline wchar_t *
UniStrcat(wchar_t *ucs1, const wchar_t *ucs2)
{
	wchar_t *anchor = ucs1;	/* save a pointer to start of ucs1 */

	while (*ucs1++) ;	/* To end of first string */
	ucs1--;			/* Return to the null */
	while ((*ucs1++ = *ucs2++)) ;	/* copy string 2 over */
	return anchor;
}

static inline wchar_t *
UniStrchr(const wchar_t *ucs, wchar_t uc)
{
	while ((*ucs != uc) && *ucs)
		ucs++;

	if (*ucs == uc)
		return (wchar_t *) ucs;
	return NULL;
}

static inline int
UniStrcmp(const wchar_t *ucs1, const wchar_t *ucs2)
{
	while ((*ucs1 == *ucs2) && *ucs1) {
		ucs1++;
		ucs2++;
	}
	return (int) *ucs1 - (int) *ucs2;
}

static inline wchar_t *
UniStrcpy(wchar_t *ucs1, const wchar_t *ucs2)
{
	wchar_t *anchor = ucs1;	/* save the start of result string */

	while ((*ucs1++ = *ucs2++)) ;
	return anchor;
}

static inline size_t
UniStrlen(const wchar_t *ucs1)
{
	int i = 0;

	while (*ucs1++)
		i++;
	return i;
}

static inline size_t
UniStrnlen(const wchar_t *ucs1, int maxlen)
{
	int i = 0;

	while (*ucs1++) {
		i++;
		if (i >= maxlen)
			break;
	}
	return i;
}

static inline wchar_t *
UniStrncat(wchar_t *ucs1, const wchar_t *ucs2, size_t n)
{
	wchar_t *anchor = ucs1;	/* save pointer to string 1 */

	while (*ucs1++) ;
	ucs1--;			/* point to null terminator of s1 */
	while (n-- && (*ucs1 = *ucs2)) {	/* copy s2 after s1 */
		ucs1++;
		ucs2++;
	}
	*ucs1 = 0;		/* Null terminate the result */
	return (anchor);
}

static inline int
UniStrncmp(const wchar_t *ucs1, const wchar_t *ucs2, size_t n)
{
	if (!n)
		return 0;	/* Null strings are equal */
	while ((*ucs1 == *ucs2) && *ucs1 && --n) {
		ucs1++;
		ucs2++;
	}
	return (int) *ucs1 - (int) *ucs2;
}

static inline int
UniStrncmp_le(const wchar_t *ucs1, const wchar_t *ucs2, size_t n)
{
	if (!n)
		return 0;	/* Null strings are equal */
	while ((*ucs1 == __le16_to_cpu(*ucs2)) && *ucs1 && --n) {
		ucs1++;
		ucs2++;
	}
	return (int) *ucs1 - (int) __le16_to_cpu(*ucs2);
}

static inline wchar_t *
UniStrncpy(wchar_t *ucs1, const wchar_t *ucs2, size_t n)
{
	wchar_t *anchor = ucs1;

	while (n-- && *ucs2)	/* Copy the strings */
		*ucs1++ = *ucs2++;

	n++;
	while (n--)		/* Pad with nulls */
		*ucs1++ = 0;
	return anchor;
}

static inline wchar_t *
UniStrncpy_le(wchar_t *ucs1, const wchar_t *ucs2, size_t n)
{
	wchar_t *anchor = ucs1;

	while (n-- && *ucs2)	/* Copy the strings */
		*ucs1++ = __le16_to_cpu(*ucs2++);

	n++;
	while (n--)		/* Pad with nulls */
		*ucs1++ = 0;
	return anchor;
}

static inline wchar_t *
UniStrstr(const wchar_t *ucs1, const wchar_t *ucs2)
{
	const wchar_t *anchor1 = ucs1;
	const wchar_t *anchor2 = ucs2;

	while (*ucs1) {
		if (*ucs1 == *ucs2) {
			/* Partial match found */
			ucs1++;
			ucs2++;
		} else {
			if (!*ucs2)	/* Match found */
				return (wchar_t *) anchor1;
			ucs1 = ++anchor1;	/* No match */
			ucs2 = anchor2;
		}
	}

	if (!*ucs2)		/* Both end together */
		return (wchar_t *) anchor1;	/* Match found */
	return NULL;		/* No match */
}

#ifndef UNIUPR_NOUPPER
static inline wchar_t
UniToupper(register wchar_t uc)
{
	register const struct UniCaseRange *rp;

	if (uc < sizeof(CifsUniUpperTable)) {
		/* Latin characters */
		return uc + CifsUniUpperTable[uc];	/* Use base tables */
	} else {
		rp = CifsUniUpperRange;	/* Use range tables */
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

static inline wchar_t *
UniStrupr(register wchar_t *upin)
{
	register wchar_t *up;

	up = upin;
	while (*up) {		/* For all characters */
		*up = UniToupper(*up);
		up++;
	}
	return upin;		/* Return input pointer */
}
#endif				/* UNIUPR_NOUPPER */

#ifndef UNIUPR_NOLOWER
static inline wchar_t
UniTolower(wchar_t uc)
{
	register struct UniCaseRange *rp;

	if (uc < sizeof(UniLowerTable)) {
		/* Latin characters */
		return uc + UniLowerTable[uc];	/* Use base tables */
	} else {
		rp = UniLowerRange;	/* Use range tables */
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

static inline wchar_t *
UniStrlwr(register wchar_t *upin)
{
	register wchar_t *up;

	up = upin;
	while (*up) {		/* For all characters */
		*up = UniTolower(*up);
		up++;
	}
	return upin;		/* Return input pointer */
}

#endif

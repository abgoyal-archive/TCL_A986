

#ifndef _P80211META_H
#define _P80211META_H

/*----------------------------------------------------------------*/
/* The following structure types are used for the metadata */
/* representation of category list metadata, group list metadata, */
/* and data item metadata for both Mib and Messages. */

typedef struct p80211meta {
	char *name;		/* data item name */
	u32 did;		/* partial did */
	u32 flags;		/* set of various flag bits */
	u32 min;		/* min value of a BOUNDEDint */
	u32 max;		/* max value of a BOUNDEDint */

	u32 maxlen;		/* maxlen of a OCTETSTR or DISPLAYSTR */
	u32 minlen;		/* minlen of a OCTETSTR or DISPLAYSTR */
	p80211enum_t *enumptr;	/* ptr to the enum type for ENUMint */
	p80211_totext_t totextptr;	/* ptr to totext conversion function */
	p80211_fromtext_t fromtextptr;	/* ptr to totext conversion function */
	p80211_valid_t validfunptr;	/* ptr to totext conversion function */
} p80211meta_t;

typedef struct grplistitem {
	char *name;
	p80211meta_t *itemlist;
} grplistitem_t;

typedef struct catlistitem {
	char *name;
	grplistitem_t *grplist;
} catlistitem_t;

#endif /* _P80211META_H */

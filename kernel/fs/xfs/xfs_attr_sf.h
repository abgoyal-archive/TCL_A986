
#ifndef __XFS_ATTR_SF_H__
#define	__XFS_ATTR_SF_H__


typedef struct xfs_attr_shortform {
	struct xfs_attr_sf_hdr {	/* constant-structure header block */
		__be16	totsize;	/* total bytes in shortform list */
		__u8	count;	/* count of active entries */
	} hdr;
	struct xfs_attr_sf_entry {
		__uint8_t namelen;	/* actual length of name (no NULL) */
		__uint8_t valuelen;	/* actual length of value (no NULL) */
		__uint8_t flags;	/* flags bits (see xfs_attr_leaf.h) */
		__uint8_t nameval[1];	/* name & value bytes concatenated */
	} list[1];			/* variable sized array */
} xfs_attr_shortform_t;
typedef struct xfs_attr_sf_hdr xfs_attr_sf_hdr_t;
typedef struct xfs_attr_sf_entry xfs_attr_sf_entry_t;

typedef struct xfs_attr_sf_sort {
	__uint8_t	entno;		/* entry number in original list */
	__uint8_t	namelen;	/* length of name value (no null) */
	__uint8_t	valuelen;	/* length of value */
	__uint8_t	flags;		/* flags bits (see xfs_attr_leaf.h) */
	xfs_dahash_t	hash;		/* this entry's hash value */
	unsigned char	*name;		/* name value, pointer into buffer */
} xfs_attr_sf_sort_t;

#define XFS_ATTR_SF_ENTSIZE_BYNAME(nlen,vlen)	/* space name/value uses */ \
	(((int)sizeof(xfs_attr_sf_entry_t)-1 + (nlen)+(vlen)))
#define XFS_ATTR_SF_ENTSIZE_MAX			/* max space for name&value */ \
	((1 << (NBBY*(int)sizeof(__uint8_t))) - 1)
#define XFS_ATTR_SF_ENTSIZE(sfep)		/* space an entry uses */ \
	((int)sizeof(xfs_attr_sf_entry_t)-1 + (sfep)->namelen+(sfep)->valuelen)
#define XFS_ATTR_SF_NEXTENTRY(sfep)		/* next entry in struct */ \
	((xfs_attr_sf_entry_t *)((char *)(sfep) + XFS_ATTR_SF_ENTSIZE(sfep)))
#define XFS_ATTR_SF_TOTSIZE(dp)			/* total space in use */ \
	(be16_to_cpu(((xfs_attr_shortform_t *)	\
		((dp)->i_afp->if_u1.if_data))->hdr.totsize))

#endif	/* __XFS_ATTR_SF_H__ */


#ifndef __XFS_ATTR_H__
#define	__XFS_ATTR_H__

struct xfs_inode;
struct xfs_da_args;
struct xfs_attr_list_context;




#define ATTR_DONTFOLLOW	0x0001	/* -- unused, from IRIX -- */
#define ATTR_ROOT	0x0002	/* use attrs in root (trusted) namespace */
#define ATTR_TRUST	0x0004	/* -- unused, from IRIX -- */
#define ATTR_SECURE	0x0008	/* use attrs in security namespace */
#define ATTR_CREATE	0x0010	/* pure create: fail if attr already exists */
#define ATTR_REPLACE	0x0020	/* pure set: fail if attr does not exist */

#define ATTR_KERNOTIME	0x1000	/* [kernel] don't update inode timestamps */
#define ATTR_KERNOVAL	0x2000	/* [kernel] get attr size only, not value */

#define XFS_ATTR_FLAGS \
	{ ATTR_DONTFOLLOW, 	"DONTFOLLOW" }, \
	{ ATTR_ROOT,		"ROOT" }, \
	{ ATTR_TRUST,		"TRUST" }, \
	{ ATTR_SECURE,		"SECURE" }, \
	{ ATTR_CREATE,		"CREATE" }, \
	{ ATTR_REPLACE,		"REPLACE" }, \
	{ ATTR_KERNOTIME,	"KERNOTIME" }, \
	{ ATTR_KERNOVAL,	"KERNOVAL" }

#define	ATTR_MAX_VALUELEN	(64*1024)	/* max length of a value */

typedef struct attrlist {
	__s32	al_count;	/* number of entries in attrlist */
	__s32	al_more;	/* T/F: more attrs (do call again) */
	__s32	al_offset[1];	/* byte offsets of attrs [var-sized] */
} attrlist_t;

typedef struct attrlist_ent {	/* data from attr_list() */
	__u32	a_valuelen;	/* number bytes in value of attr */
	char	a_name[1];	/* attr name (NULL terminated) */
} attrlist_ent_t;

#define	ATTR_ENTRY(buffer, index)		\
	((attrlist_ent_t *)			\
	 &((char *)buffer)[ ((attrlist_t *)(buffer))->al_offset[index] ])

typedef struct attrlist_cursor_kern {
	__u32	hashval;	/* hash value of next entry to add */
	__u32	blkno;		/* block containing entry (suggestion) */
	__u32	offset;		/* offset in list of equal-hashvals */
	__u16	pad1;		/* padding to match user-level */
	__u8	pad2;		/* padding to match user-level */
	__u8	initted;	/* T/F: cursor has been initialized */
} attrlist_cursor_kern_t;




typedef int (*put_listent_func_t)(struct xfs_attr_list_context *, int,
			      unsigned char *, int, int, unsigned char *);

typedef struct xfs_attr_list_context {
	struct xfs_inode		*dp;		/* inode */
	struct attrlist_cursor_kern	*cursor;	/* position in list */
	char				*alist;		/* output buffer */
	int				seen_enough;	/* T/F: seen enough of list? */
	ssize_t				count;		/* num used entries */
	int				dupcnt;		/* count dup hashvals seen */
	int				bufsize;	/* total buffer size */
	int				firstu;		/* first used byte in buffer */
	int				flags;		/* from VOP call */
	int				resynch;	/* T/F: resynch with cursor */
	int				put_value;	/* T/F: need value for listent */
	put_listent_func_t		put_listent;	/* list output fmt function */
	int				index;		/* index into output buffer */
} xfs_attr_list_context_t;



int xfs_attr_inactive(struct xfs_inode *dp);
int xfs_attr_rmtval_get(struct xfs_da_args *args);
int xfs_attr_list_int(struct xfs_attr_list_context *);

#endif	/* __XFS_ATTR_H__ */


#ifndef	__XFS_EXTFREE_ITEM_H__
#define	__XFS_EXTFREE_ITEM_H__

struct xfs_mount;
struct kmem_zone;

typedef struct xfs_extent {
	xfs_dfsbno_t	ext_start;
	xfs_extlen_t	ext_len;
} xfs_extent_t;


typedef struct xfs_extent_32 {
	__uint64_t	ext_start;
	__uint32_t	ext_len;
} __attribute__((packed)) xfs_extent_32_t;

typedef struct xfs_extent_64 {
	__uint64_t	ext_start;
	__uint32_t	ext_len;
	__uint32_t	ext_pad;
} xfs_extent_64_t;

typedef struct xfs_efi_log_format {
	__uint16_t		efi_type;	/* efi log item type */
	__uint16_t		efi_size;	/* size of this item */
	__uint32_t		efi_nextents;	/* # extents to free */
	__uint64_t		efi_id;		/* efi identifier */
	xfs_extent_t		efi_extents[1];	/* array of extents to free */
} xfs_efi_log_format_t;

typedef struct xfs_efi_log_format_32 {
	__uint16_t		efi_type;	/* efi log item type */
	__uint16_t		efi_size;	/* size of this item */
	__uint32_t		efi_nextents;	/* # extents to free */
	__uint64_t		efi_id;		/* efi identifier */
	xfs_extent_32_t		efi_extents[1];	/* array of extents to free */
} __attribute__((packed)) xfs_efi_log_format_32_t;

typedef struct xfs_efi_log_format_64 {
	__uint16_t		efi_type;	/* efi log item type */
	__uint16_t		efi_size;	/* size of this item */
	__uint32_t		efi_nextents;	/* # extents to free */
	__uint64_t		efi_id;		/* efi identifier */
	xfs_extent_64_t		efi_extents[1];	/* array of extents to free */
} xfs_efi_log_format_64_t;

typedef struct xfs_efd_log_format {
	__uint16_t		efd_type;	/* efd log item type */
	__uint16_t		efd_size;	/* size of this item */
	__uint32_t		efd_nextents;	/* # of extents freed */
	__uint64_t		efd_efi_id;	/* id of corresponding efi */
	xfs_extent_t		efd_extents[1];	/* array of extents freed */
} xfs_efd_log_format_t;

typedef struct xfs_efd_log_format_32 {
	__uint16_t		efd_type;	/* efd log item type */
	__uint16_t		efd_size;	/* size of this item */
	__uint32_t		efd_nextents;	/* # of extents freed */
	__uint64_t		efd_efi_id;	/* id of corresponding efi */
	xfs_extent_32_t		efd_extents[1];	/* array of extents freed */
} __attribute__((packed)) xfs_efd_log_format_32_t;

typedef struct xfs_efd_log_format_64 {
	__uint16_t		efd_type;	/* efd log item type */
	__uint16_t		efd_size;	/* size of this item */
	__uint32_t		efd_nextents;	/* # of extents freed */
	__uint64_t		efd_efi_id;	/* id of corresponding efi */
	xfs_extent_64_t		efd_extents[1];	/* array of extents freed */
} xfs_efd_log_format_64_t;


#ifdef __KERNEL__

#define	XFS_EFI_MAX_FAST_EXTENTS	16

#define	XFS_EFI_RECOVERED	0x1
#define	XFS_EFI_COMMITTED	0x2
#define	XFS_EFI_CANCELED	0x4

typedef struct xfs_efi_log_item {
	xfs_log_item_t		efi_item;
	uint			efi_flags;	/* misc flags */
	uint			efi_next_extent;
	xfs_efi_log_format_t	efi_format;
} xfs_efi_log_item_t;

typedef struct xfs_efd_log_item {
	xfs_log_item_t		efd_item;
	xfs_efi_log_item_t	*efd_efip;
	uint			efd_next_extent;
	xfs_efd_log_format_t	efd_format;
} xfs_efd_log_item_t;

#define	XFS_EFD_MAX_FAST_EXTENTS	16

extern struct kmem_zone	*xfs_efi_zone;
extern struct kmem_zone	*xfs_efd_zone;

xfs_efi_log_item_t	*xfs_efi_init(struct xfs_mount *, uint);
xfs_efd_log_item_t	*xfs_efd_init(struct xfs_mount *, xfs_efi_log_item_t *,
				      uint);
int			xfs_efi_copy_format(xfs_log_iovec_t *buf,
					    xfs_efi_log_format_t *dst_efi_fmt);
void			xfs_efi_item_free(xfs_efi_log_item_t *);

#endif	/* __KERNEL__ */

#endif	/* __XFS_EXTFREE_ITEM_H__ */

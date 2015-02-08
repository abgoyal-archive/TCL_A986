
#include "xfs.h"
#include "xfs_fs.h"
#include "xfs_types.h"
#include "xfs_log.h"
#include "xfs_inum.h"
#include "xfs_trans.h"
#include "xfs_buf_item.h"
#include "xfs_sb.h"
#include "xfs_ag.h"
#include "xfs_dmapi.h"
#include "xfs_mount.h"
#include "xfs_trans_priv.h"
#include "xfs_extfree_item.h"


kmem_zone_t	*xfs_efi_zone;
kmem_zone_t	*xfs_efd_zone;

STATIC void	xfs_efi_item_unlock(xfs_efi_log_item_t *);

void
xfs_efi_item_free(xfs_efi_log_item_t *efip)
{
	int nexts = efip->efi_format.efi_nextents;

	if (nexts > XFS_EFI_MAX_FAST_EXTENTS) {
		kmem_free(efip);
	} else {
		kmem_zone_free(xfs_efi_zone, efip);
	}
}

/*ARGSUSED*/
STATIC uint
xfs_efi_item_size(xfs_efi_log_item_t *efip)
{
	return 1;
}

STATIC void
xfs_efi_item_format(xfs_efi_log_item_t	*efip,
		    xfs_log_iovec_t	*log_vector)
{
	uint	size;

	ASSERT(efip->efi_next_extent == efip->efi_format.efi_nextents);

	efip->efi_format.efi_type = XFS_LI_EFI;

	size = sizeof(xfs_efi_log_format_t);
	size += (efip->efi_format.efi_nextents - 1) * sizeof(xfs_extent_t);
	efip->efi_format.efi_size = 1;

	log_vector->i_addr = (xfs_caddr_t)&(efip->efi_format);
	log_vector->i_len = size;
	log_vector->i_type = XLOG_REG_TYPE_EFI_FORMAT;
	ASSERT(size >= sizeof(xfs_efi_log_format_t));
}


/*ARGSUSED*/
STATIC void
xfs_efi_item_pin(xfs_efi_log_item_t *efip)
{
	return;
}


/*ARGSUSED*/
STATIC void
xfs_efi_item_unpin(xfs_efi_log_item_t *efip)
{
	struct xfs_ail		*ailp = efip->efi_item.li_ailp;

	spin_lock(&ailp->xa_lock);
	if (efip->efi_flags & XFS_EFI_CANCELED) {
		/* xfs_trans_ail_delete() drops the AIL lock. */
		xfs_trans_ail_delete(ailp, (xfs_log_item_t *)efip);
		xfs_efi_item_free(efip);
	} else {
		efip->efi_flags |= XFS_EFI_COMMITTED;
		spin_unlock(&ailp->xa_lock);
	}
}

STATIC void
xfs_efi_item_unpin_remove(xfs_efi_log_item_t *efip, xfs_trans_t *tp)
{
	struct xfs_ail		*ailp = efip->efi_item.li_ailp;
	xfs_log_item_desc_t	*lidp;

	spin_lock(&ailp->xa_lock);
	if (efip->efi_flags & XFS_EFI_CANCELED) {
		/*
		 * free the xaction descriptor pointing to this item
		 */
		lidp = xfs_trans_find_item(tp, (xfs_log_item_t *) efip);
		xfs_trans_free_item(tp, lidp);

		/* xfs_trans_ail_delete() drops the AIL lock. */
		xfs_trans_ail_delete(ailp, (xfs_log_item_t *)efip);
		xfs_efi_item_free(efip);
	} else {
		efip->efi_flags |= XFS_EFI_COMMITTED;
		spin_unlock(&ailp->xa_lock);
	}
}

/*ARGSUSED*/
STATIC uint
xfs_efi_item_trylock(xfs_efi_log_item_t *efip)
{
	return XFS_ITEM_PINNED;
}

/*ARGSUSED*/
STATIC void
xfs_efi_item_unlock(xfs_efi_log_item_t *efip)
{
	if (efip->efi_item.li_flags & XFS_LI_ABORTED)
		xfs_efi_item_free(efip);
	return;
}

/*ARGSUSED*/
STATIC xfs_lsn_t
xfs_efi_item_committed(xfs_efi_log_item_t *efip, xfs_lsn_t lsn)
{
	return lsn;
}

/*ARGSUSED*/
STATIC void
xfs_efi_item_push(xfs_efi_log_item_t *efip)
{
	return;
}

/*ARGSUSED*/
STATIC void
xfs_efi_item_committing(xfs_efi_log_item_t *efip, xfs_lsn_t lsn)
{
	return;
}

static struct xfs_item_ops xfs_efi_item_ops = {
	.iop_size	= (uint(*)(xfs_log_item_t*))xfs_efi_item_size,
	.iop_format	= (void(*)(xfs_log_item_t*, xfs_log_iovec_t*))
					xfs_efi_item_format,
	.iop_pin	= (void(*)(xfs_log_item_t*))xfs_efi_item_pin,
	.iop_unpin	= (void(*)(xfs_log_item_t*))xfs_efi_item_unpin,
	.iop_unpin_remove = (void(*)(xfs_log_item_t*, xfs_trans_t *))
					xfs_efi_item_unpin_remove,
	.iop_trylock	= (uint(*)(xfs_log_item_t*))xfs_efi_item_trylock,
	.iop_unlock	= (void(*)(xfs_log_item_t*))xfs_efi_item_unlock,
	.iop_committed	= (xfs_lsn_t(*)(xfs_log_item_t*, xfs_lsn_t))
					xfs_efi_item_committed,
	.iop_push	= (void(*)(xfs_log_item_t*))xfs_efi_item_push,
	.iop_pushbuf	= NULL,
	.iop_committing = (void(*)(xfs_log_item_t*, xfs_lsn_t))
					xfs_efi_item_committing
};


xfs_efi_log_item_t *
xfs_efi_init(xfs_mount_t	*mp,
	     uint		nextents)

{
	xfs_efi_log_item_t	*efip;
	uint			size;

	ASSERT(nextents > 0);
	if (nextents > XFS_EFI_MAX_FAST_EXTENTS) {
		size = (uint)(sizeof(xfs_efi_log_item_t) +
			((nextents - 1) * sizeof(xfs_extent_t)));
		efip = (xfs_efi_log_item_t*)kmem_zalloc(size, KM_SLEEP);
	} else {
		efip = (xfs_efi_log_item_t*)kmem_zone_zalloc(xfs_efi_zone,
							     KM_SLEEP);
	}

	xfs_log_item_init(mp, &efip->efi_item, XFS_LI_EFI, &xfs_efi_item_ops);
	efip->efi_format.efi_nextents = nextents;
	efip->efi_format.efi_id = (__psint_t)(void*)efip;

	return (efip);
}

int
xfs_efi_copy_format(xfs_log_iovec_t *buf, xfs_efi_log_format_t *dst_efi_fmt)
{
	xfs_efi_log_format_t *src_efi_fmt = (xfs_efi_log_format_t *)buf->i_addr;
	uint i;
	uint len = sizeof(xfs_efi_log_format_t) + 
		(src_efi_fmt->efi_nextents - 1) * sizeof(xfs_extent_t);  
	uint len32 = sizeof(xfs_efi_log_format_32_t) + 
		(src_efi_fmt->efi_nextents - 1) * sizeof(xfs_extent_32_t);  
	uint len64 = sizeof(xfs_efi_log_format_64_t) + 
		(src_efi_fmt->efi_nextents - 1) * sizeof(xfs_extent_64_t);  

	if (buf->i_len == len) {
		memcpy((char *)dst_efi_fmt, (char*)src_efi_fmt, len);
		return 0;
	} else if (buf->i_len == len32) {
		xfs_efi_log_format_32_t *src_efi_fmt_32 =
			(xfs_efi_log_format_32_t *)buf->i_addr;

		dst_efi_fmt->efi_type     = src_efi_fmt_32->efi_type;
		dst_efi_fmt->efi_size     = src_efi_fmt_32->efi_size;
		dst_efi_fmt->efi_nextents = src_efi_fmt_32->efi_nextents;
		dst_efi_fmt->efi_id       = src_efi_fmt_32->efi_id;
		for (i = 0; i < dst_efi_fmt->efi_nextents; i++) {
			dst_efi_fmt->efi_extents[i].ext_start =
				src_efi_fmt_32->efi_extents[i].ext_start;
			dst_efi_fmt->efi_extents[i].ext_len =
				src_efi_fmt_32->efi_extents[i].ext_len;
		}
		return 0;
	} else if (buf->i_len == len64) {
		xfs_efi_log_format_64_t *src_efi_fmt_64 =
			(xfs_efi_log_format_64_t *)buf->i_addr;

		dst_efi_fmt->efi_type     = src_efi_fmt_64->efi_type;
		dst_efi_fmt->efi_size     = src_efi_fmt_64->efi_size;
		dst_efi_fmt->efi_nextents = src_efi_fmt_64->efi_nextents;
		dst_efi_fmt->efi_id       = src_efi_fmt_64->efi_id;
		for (i = 0; i < dst_efi_fmt->efi_nextents; i++) {
			dst_efi_fmt->efi_extents[i].ext_start =
				src_efi_fmt_64->efi_extents[i].ext_start;
			dst_efi_fmt->efi_extents[i].ext_len =
				src_efi_fmt_64->efi_extents[i].ext_len;
		}
		return 0;
	}
	return EFSCORRUPTED;
}

void
xfs_efi_release(xfs_efi_log_item_t	*efip,
		uint			nextents)
{
	struct xfs_ail		*ailp = efip->efi_item.li_ailp;
	int			extents_left;

	ASSERT(efip->efi_next_extent > 0);
	ASSERT(efip->efi_flags & XFS_EFI_COMMITTED);

	spin_lock(&ailp->xa_lock);
	ASSERT(efip->efi_next_extent >= nextents);
	efip->efi_next_extent -= nextents;
	extents_left = efip->efi_next_extent;
	if (extents_left == 0) {
		/* xfs_trans_ail_delete() drops the AIL lock. */
		xfs_trans_ail_delete(ailp, (xfs_log_item_t *)efip);
		xfs_efi_item_free(efip);
	} else {
		spin_unlock(&ailp->xa_lock);
	}
}

STATIC void
xfs_efd_item_free(xfs_efd_log_item_t *efdp)
{
	int nexts = efdp->efd_format.efd_nextents;

	if (nexts > XFS_EFD_MAX_FAST_EXTENTS) {
		kmem_free(efdp);
	} else {
		kmem_zone_free(xfs_efd_zone, efdp);
	}
}

/*ARGSUSED*/
STATIC uint
xfs_efd_item_size(xfs_efd_log_item_t *efdp)
{
	return 1;
}

STATIC void
xfs_efd_item_format(xfs_efd_log_item_t	*efdp,
		    xfs_log_iovec_t	*log_vector)
{
	uint	size;

	ASSERT(efdp->efd_next_extent == efdp->efd_format.efd_nextents);

	efdp->efd_format.efd_type = XFS_LI_EFD;

	size = sizeof(xfs_efd_log_format_t);
	size += (efdp->efd_format.efd_nextents - 1) * sizeof(xfs_extent_t);
	efdp->efd_format.efd_size = 1;

	log_vector->i_addr = (xfs_caddr_t)&(efdp->efd_format);
	log_vector->i_len = size;
	log_vector->i_type = XLOG_REG_TYPE_EFD_FORMAT;
	ASSERT(size >= sizeof(xfs_efd_log_format_t));
}


/*ARGSUSED*/
STATIC void
xfs_efd_item_pin(xfs_efd_log_item_t *efdp)
{
	return;
}


/*ARGSUSED*/
STATIC void
xfs_efd_item_unpin(xfs_efd_log_item_t *efdp)
{
	return;
}

/*ARGSUSED*/
STATIC void
xfs_efd_item_unpin_remove(xfs_efd_log_item_t *efdp, xfs_trans_t *tp)
{
	return;
}

/*ARGSUSED*/
STATIC uint
xfs_efd_item_trylock(xfs_efd_log_item_t *efdp)
{
	return XFS_ITEM_LOCKED;
}

/*ARGSUSED*/
STATIC void
xfs_efd_item_unlock(xfs_efd_log_item_t *efdp)
{
	if (efdp->efd_item.li_flags & XFS_LI_ABORTED)
		xfs_efd_item_free(efdp);
	return;
}

/*ARGSUSED*/
STATIC xfs_lsn_t
xfs_efd_item_committed(xfs_efd_log_item_t *efdp, xfs_lsn_t lsn)
{
	/*
	 * If we got a log I/O error, it's always the case that the LR with the
	 * EFI got unpinned and freed before the EFD got aborted.
	 */
	if ((efdp->efd_item.li_flags & XFS_LI_ABORTED) == 0)
		xfs_efi_release(efdp->efd_efip, efdp->efd_format.efd_nextents);

	xfs_efd_item_free(efdp);
	return (xfs_lsn_t)-1;
}

/*ARGSUSED*/
STATIC void
xfs_efd_item_push(xfs_efd_log_item_t *efdp)
{
	return;
}

/*ARGSUSED*/
STATIC void
xfs_efd_item_committing(xfs_efd_log_item_t *efip, xfs_lsn_t lsn)
{
	return;
}

static struct xfs_item_ops xfs_efd_item_ops = {
	.iop_size	= (uint(*)(xfs_log_item_t*))xfs_efd_item_size,
	.iop_format	= (void(*)(xfs_log_item_t*, xfs_log_iovec_t*))
					xfs_efd_item_format,
	.iop_pin	= (void(*)(xfs_log_item_t*))xfs_efd_item_pin,
	.iop_unpin	= (void(*)(xfs_log_item_t*))xfs_efd_item_unpin,
	.iop_unpin_remove = (void(*)(xfs_log_item_t*, xfs_trans_t*))
					xfs_efd_item_unpin_remove,
	.iop_trylock	= (uint(*)(xfs_log_item_t*))xfs_efd_item_trylock,
	.iop_unlock	= (void(*)(xfs_log_item_t*))xfs_efd_item_unlock,
	.iop_committed	= (xfs_lsn_t(*)(xfs_log_item_t*, xfs_lsn_t))
					xfs_efd_item_committed,
	.iop_push	= (void(*)(xfs_log_item_t*))xfs_efd_item_push,
	.iop_pushbuf	= NULL,
	.iop_committing = (void(*)(xfs_log_item_t*, xfs_lsn_t))
					xfs_efd_item_committing
};


xfs_efd_log_item_t *
xfs_efd_init(xfs_mount_t	*mp,
	     xfs_efi_log_item_t	*efip,
	     uint		nextents)

{
	xfs_efd_log_item_t	*efdp;
	uint			size;

	ASSERT(nextents > 0);
	if (nextents > XFS_EFD_MAX_FAST_EXTENTS) {
		size = (uint)(sizeof(xfs_efd_log_item_t) +
			((nextents - 1) * sizeof(xfs_extent_t)));
		efdp = (xfs_efd_log_item_t*)kmem_zalloc(size, KM_SLEEP);
	} else {
		efdp = (xfs_efd_log_item_t*)kmem_zone_zalloc(xfs_efd_zone,
							     KM_SLEEP);
	}

	xfs_log_item_init(mp, &efdp->efd_item, XFS_LI_EFD, &xfs_efd_item_ops);
	efdp->efd_efip = efip;
	efdp->efd_format.efd_nextents = nextents;
	efdp->efd_format.efd_efi_id = efip->efi_format.efi_id;

	return (efdp);
}

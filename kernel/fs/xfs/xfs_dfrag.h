
#ifndef __XFS_DFRAG_H__
#define	__XFS_DFRAG_H__


typedef struct xfs_swapext
{
	__int64_t	sx_version;	/* version */
	__int64_t	sx_fdtarget;	/* fd of target file */
	__int64_t	sx_fdtmp;	/* fd of tmp file */
	xfs_off_t	sx_offset;	/* offset into file */
	xfs_off_t	sx_length;	/* leng from offset */
	char		sx_pad[16];	/* pad space, unused */
	xfs_bstat_t	sx_stat;	/* stat of target b4 copy */
} xfs_swapext_t;

#define XFS_SX_VERSION		0

#ifdef __KERNEL__

int	xfs_swapext(struct xfs_swapext *sx);

#endif	/* __KERNEL__ */

#endif	/* __XFS_DFRAG_H__ */


#ifndef __XFS_DMAPI_PRIV_H__
#define __XFS_DMAPI_PRIV_H__

#define DM_SEM_FLAG_RD(ioflags) (((ioflags) & IO_ISDIRECT) ? \
			      DM_FLAGS_IMUX : 0)
#define DM_SEM_FLAG_WR	(DM_FLAGS_IALLOCSEM_WR | DM_FLAGS_IMUX)

#endif /*__XFS_DMAPI_PRIV_H__*/

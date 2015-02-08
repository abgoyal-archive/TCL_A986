

#ifndef _LINUX_NTFS_QUOTA_H
#define _LINUX_NTFS_QUOTA_H

#ifdef NTFS_RW

#include "types.h"
#include "volume.h"

extern bool ntfs_mark_quotas_out_of_date(ntfs_volume *vol);

#endif /* NTFS_RW */

#endif /* _LINUX_NTFS_QUOTA_H */

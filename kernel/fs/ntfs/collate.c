

#include "collate.h"
#include "debug.h"
#include "ntfs.h"

static int ntfs_collate_binary(ntfs_volume *vol,
		const void *data1, const int data1_len,
		const void *data2, const int data2_len)
{
	int rc;

	ntfs_debug("Entering.");
	rc = memcmp(data1, data2, min(data1_len, data2_len));
	if (!rc && (data1_len != data2_len)) {
		if (data1_len < data2_len)
			rc = -1;
		else
			rc = 1;
	}
	ntfs_debug("Done, returning %i", rc);
	return rc;
}

static int ntfs_collate_ntofs_ulong(ntfs_volume *vol,
		const void *data1, const int data1_len,
		const void *data2, const int data2_len)
{
	int rc;
	u32 d1, d2;

	ntfs_debug("Entering.");
	// FIXME:  We don't really want to bug here.
	BUG_ON(data1_len != data2_len);
	BUG_ON(data1_len != 4);
	d1 = le32_to_cpup(data1);
	d2 = le32_to_cpup(data2);
	if (d1 < d2)
		rc = -1;
	else {
		if (d1 == d2)
			rc = 0;
		else
			rc = 1;
	}
	ntfs_debug("Done, returning %i", rc);
	return rc;
}

typedef int (*ntfs_collate_func_t)(ntfs_volume *, const void *, const int,
		const void *, const int);

static ntfs_collate_func_t ntfs_do_collate0x0[3] = {
	ntfs_collate_binary,
	NULL/*ntfs_collate_file_name*/,
	NULL/*ntfs_collate_unicode_string*/,
};

static ntfs_collate_func_t ntfs_do_collate0x1[4] = {
	ntfs_collate_ntofs_ulong,
	NULL/*ntfs_collate_ntofs_sid*/,
	NULL/*ntfs_collate_ntofs_security_hash*/,
	NULL/*ntfs_collate_ntofs_ulongs*/,
};

int ntfs_collate(ntfs_volume *vol, COLLATION_RULE cr,
		const void *data1, const int data1_len,
		const void *data2, const int data2_len) {
	int i;

	ntfs_debug("Entering.");
	/*
	 * FIXME:  At the moment we only support COLLATION_BINARY and
	 * COLLATION_NTOFS_ULONG, so we BUG() for everything else for now.
	 */
	BUG_ON(cr != COLLATION_BINARY && cr != COLLATION_NTOFS_ULONG);
	i = le32_to_cpu(cr);
	BUG_ON(i < 0);
	if (i <= 0x02)
		return ntfs_do_collate0x0[i](vol, data1, data1_len,
				data2, data2_len);
	BUG_ON(i < 0x10);
	i -= 0x10;
	if (likely(i <= 3))
		return ntfs_do_collate0x1[i](vol, data1, data1_len,
				data2, data2_len);
	BUG();
	return 0;
}

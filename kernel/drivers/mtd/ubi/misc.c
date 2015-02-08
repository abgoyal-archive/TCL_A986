

/* Here we keep miscellaneous functions which are used all over the UBI code */

#include "ubi.h"

int ubi_calc_data_len(const struct ubi_device *ubi, const void *buf,
		      int length)
{
	int i;

	ubi_assert(!(length & (ubi->min_io_size - 1)));

	for (i = length - 1; i >= 0; i--)
		if (((const uint8_t *)buf)[i] != 0xFF)
			break;

	/* The resulting length must be aligned to the minimum flash I/O size */
	length = ALIGN(i + 1, ubi->min_io_size);
	return length;
}

int ubi_check_volume(struct ubi_device *ubi, int vol_id)
{
	void *buf;
	int err = 0, i;
	struct ubi_volume *vol = ubi->volumes[vol_id];

	if (vol->vol_type != UBI_STATIC_VOLUME)
		return 0;

	buf = vmalloc(vol->usable_leb_size);
	if (!buf)
		return -ENOMEM;

	for (i = 0; i < vol->used_ebs; i++) {
		int size;

		if (i == vol->used_ebs - 1)
			size = vol->last_eb_bytes;
		else
			size = vol->usable_leb_size;

		err = ubi_eba_read_leb(ubi, vol, i, buf, 0, size, 1);
		if (err) {
			if (err == -EBADMSG)
				err = 1;
			break;
		}
	}

	vfree(buf);
	return err;
}

void ubi_calculate_reserved(struct ubi_device *ubi)
{
	ubi->beb_rsvd_level = ubi->good_peb_count/100;
	ubi->beb_rsvd_level *= CONFIG_MTD_UBI_BEB_RESERVE;
	if (ubi->beb_rsvd_level < MIN_RESEVED_PEBS)
		ubi->beb_rsvd_level = MIN_RESEVED_PEBS;
}

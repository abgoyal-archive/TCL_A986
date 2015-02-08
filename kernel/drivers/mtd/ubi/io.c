


#include <linux/crc32.h>
#include <linux/err.h>
#include <linux/slab.h>
#include "ubi.h"

#ifdef CONFIG_MTD_UBI_DEBUG_PARANOID
static int paranoid_check_not_bad(const struct ubi_device *ubi, int pnum);
static int paranoid_check_peb_ec_hdr(const struct ubi_device *ubi, int pnum);
static int paranoid_check_ec_hdr(const struct ubi_device *ubi, int pnum,
				 const struct ubi_ec_hdr *ec_hdr);
static int paranoid_check_peb_vid_hdr(const struct ubi_device *ubi, int pnum);
static int paranoid_check_vid_hdr(const struct ubi_device *ubi, int pnum,
				  const struct ubi_vid_hdr *vid_hdr);
#else
#define paranoid_check_not_bad(ubi, pnum) 0
#define paranoid_check_peb_ec_hdr(ubi, pnum)  0
#define paranoid_check_ec_hdr(ubi, pnum, ec_hdr)  0
#define paranoid_check_peb_vid_hdr(ubi, pnum) 0
#define paranoid_check_vid_hdr(ubi, pnum, vid_hdr) 0
#endif

int ubi_io_read(const struct ubi_device *ubi, void *buf, int pnum, int offset,
		int len)
{
	int err, retries = 0;
	size_t read;
	loff_t addr;

	dbg_io("read %d bytes from PEB %d:%d", len, pnum, offset);

	ubi_assert(pnum >= 0 && pnum < ubi->peb_count);
	ubi_assert(offset >= 0 && offset + len <= ubi->peb_size);
	ubi_assert(len > 0);

	err = paranoid_check_not_bad(ubi, pnum);
	if (err)
		return err;

	addr = (loff_t)pnum * ubi->peb_size + offset;
retry:
	err = ubi->mtd->read(ubi->mtd, addr, len, &read, buf);
	if (err) {
		if (err == -EUCLEAN) {
			/*
			 * -EUCLEAN is reported if there was a bit-flip which
			 * was corrected, so this is harmless.
			 *
			 * We do not report about it here unless debugging is
			 * enabled. A corresponding message will be printed
			 * later, when it is has been scrubbed.
			 */
			dbg_msg("fixable bit-flip detected at PEB %d", pnum);
			ubi_assert(len == read);
			return UBI_IO_BITFLIPS;
		}

		if (read != len && retries++ < UBI_IO_RETRIES) {
			dbg_io("error %d while reading %d bytes from PEB %d:%d,"
			       " read only %zd bytes, retry",
			       err, len, pnum, offset, read);
			yield();
			goto retry;
		}

		ubi_err("error %d while reading %d bytes from PEB %d:%d, "
			"read %zd bytes", err, len, pnum, offset, read);
		ubi_dbg_dump_stack();

		/*
		 * The driver should never return -EBADMSG if it failed to read
		 * all the requested data. But some buggy drivers might do
		 * this, so we change it to -EIO.
		 */
		if (read != len && err == -EBADMSG) {
			ubi_assert(0);
			err = -EIO;
		}
	} else {
		ubi_assert(len == read);

		if (ubi_dbg_is_bitflip()) {
			dbg_gen("bit-flip (emulated)");
			err = UBI_IO_BITFLIPS;
		}
	}

	return err;
}

int ubi_io_write(struct ubi_device *ubi, const void *buf, int pnum, int offset,
		 int len)
{
	int err;
	size_t written;
	loff_t addr;

	dbg_io("write %d bytes to PEB %d:%d", len, pnum, offset);

	ubi_assert(pnum >= 0 && pnum < ubi->peb_count);
	ubi_assert(offset >= 0 && offset + len <= ubi->peb_size);
	ubi_assert(offset % ubi->hdrs_min_io_size == 0);
	ubi_assert(len > 0 && len % ubi->hdrs_min_io_size == 0);

	if (ubi->ro_mode) {
		ubi_err("read-only mode");
		return -EROFS;
	}

	/* The below has to be compiled out if paranoid checks are disabled */

	err = paranoid_check_not_bad(ubi, pnum);
	if (err)
		return err;

	/* The area we are writing to has to contain all 0xFF bytes */
	err = ubi_dbg_check_all_ff(ubi, pnum, offset, len);
	if (err)
		return err;

	if (offset >= ubi->leb_start) {
		/*
		 * We write to the data area of the physical eraseblock. Make
		 * sure it has valid EC and VID headers.
		 */
		err = paranoid_check_peb_ec_hdr(ubi, pnum);
		if (err)
			return err;
		err = paranoid_check_peb_vid_hdr(ubi, pnum);
		if (err)
			return err;
	}

	if (ubi_dbg_is_write_failure()) {
		dbg_err("cannot write %d bytes to PEB %d:%d "
			"(emulated)", len, pnum, offset);
		ubi_dbg_dump_stack();
		return -EIO;
	}

	addr = (loff_t)pnum * ubi->peb_size + offset;
	err = ubi->mtd->write(ubi->mtd, addr, len, &written, buf);
	if (err) {
		ubi_err("error %d while writing %d bytes to PEB %d:%d, written "
			"%zd bytes", err, len, pnum, offset, written);
		ubi_dbg_dump_stack();
		ubi_dbg_dump_flash(ubi, pnum, offset, len);
	} else
		ubi_assert(written == len);

	if (!err) {
		err = ubi_dbg_check_write(ubi, buf, pnum, offset, len);
		if (err)
			return err;

		/*
		 * Since we always write sequentially, the rest of the PEB has
		 * to contain only 0xFF bytes.
		 */
		offset += len;
		len = ubi->peb_size - offset;
		if (len)
			err = ubi_dbg_check_all_ff(ubi, pnum, offset, len);
	}

	return err;
}

static void erase_callback(struct erase_info *ei)
{
	wake_up_interruptible((wait_queue_head_t *)ei->priv);
}

static int do_sync_erase(struct ubi_device *ubi, int pnum)
{
	int err, retries = 0;
	struct erase_info ei;
	wait_queue_head_t wq;

	dbg_io("erase PEB %d", pnum);

retry:
	init_waitqueue_head(&wq);
	memset(&ei, 0, sizeof(struct erase_info));

	ei.mtd      = ubi->mtd;
	ei.addr     = (loff_t)pnum * ubi->peb_size;
	ei.len      = ubi->peb_size;
	ei.callback = erase_callback;
	ei.priv     = (unsigned long)&wq;

	err = ubi->mtd->erase(ubi->mtd, &ei);
	if (err) {
		if (retries++ < UBI_IO_RETRIES) {
			dbg_io("error %d while erasing PEB %d, retry",
			       err, pnum);
			yield();
			goto retry;
		}
		ubi_err("cannot erase PEB %d, error %d", pnum, err);
		ubi_dbg_dump_stack();
		return err;
	}

	err = wait_event_interruptible(wq, ei.state == MTD_ERASE_DONE ||
					   ei.state == MTD_ERASE_FAILED);
	if (err) {
		ubi_err("interrupted PEB %d erasure", pnum);
		return -EINTR;
	}

	if (ei.state == MTD_ERASE_FAILED) {
		if (retries++ < UBI_IO_RETRIES) {
			dbg_io("error while erasing PEB %d, retry", pnum);
			yield();
			goto retry;
		}
		ubi_err("cannot erase PEB %d", pnum);
		ubi_dbg_dump_stack();
		return -EIO;
	}

	err = ubi_dbg_check_all_ff(ubi, pnum, 0, ubi->peb_size);
	if (err)
		return err;

	if (ubi_dbg_is_erase_failure() && !err) {
		dbg_err("cannot erase PEB %d (emulated)", pnum);
		return -EIO;
	}

	return 0;
}

static int check_pattern(const void *buf, uint8_t patt, int size)
{
	int i;

	for (i = 0; i < size; i++)
		if (((const uint8_t *)buf)[i] != patt)
			return 0;
	return 1;
}

/* Patterns to write to a physical eraseblock when torturing it */
static uint8_t patterns[] = {0xa5, 0x5a, 0x0};

static int torture_peb(struct ubi_device *ubi, int pnum)
{
	int err, i, patt_count;

	ubi_msg("run torture test for PEB %d", pnum);
	patt_count = ARRAY_SIZE(patterns);
	ubi_assert(patt_count > 0);

	mutex_lock(&ubi->buf_mutex);
	for (i = 0; i < patt_count; i++) {
		err = do_sync_erase(ubi, pnum);
		if (err)
			goto out;

		/* Make sure the PEB contains only 0xFF bytes */
		err = ubi_io_read(ubi, ubi->peb_buf1, pnum, 0, ubi->peb_size);
		if (err)
			goto out;

		err = check_pattern(ubi->peb_buf1, 0xFF, ubi->peb_size);
		if (err == 0) {
			ubi_err("erased PEB %d, but a non-0xFF byte found",
				pnum);
			err = -EIO;
			goto out;
		}

		/* Write a pattern and check it */
		memset(ubi->peb_buf1, patterns[i], ubi->peb_size);
		err = ubi_io_write(ubi, ubi->peb_buf1, pnum, 0, ubi->peb_size);
		if (err)
			goto out;

		memset(ubi->peb_buf1, ~patterns[i], ubi->peb_size);
		err = ubi_io_read(ubi, ubi->peb_buf1, pnum, 0, ubi->peb_size);
		if (err)
			goto out;

		err = check_pattern(ubi->peb_buf1, patterns[i], ubi->peb_size);
		if (err == 0) {
			ubi_err("pattern %x checking failed for PEB %d",
				patterns[i], pnum);
			err = -EIO;
			goto out;
		}
	}

	err = patt_count;
	ubi_msg("PEB %d passed torture test, do not mark it a bad", pnum);

out:
	mutex_unlock(&ubi->buf_mutex);
	if (err == UBI_IO_BITFLIPS || err == -EBADMSG) {
		/*
		 * If a bit-flip or data integrity error was detected, the test
		 * has not passed because it happened on a freshly erased
		 * physical eraseblock which means something is wrong with it.
		 */
		ubi_err("read problems on freshly erased PEB %d, must be bad",
			pnum);
		err = -EIO;
	}
	return err;
}

static int nor_erase_prepare(struct ubi_device *ubi, int pnum)
{
	int err, err1;
	size_t written;
	loff_t addr;
	uint32_t data = 0;
	struct ubi_vid_hdr vid_hdr;

	addr = (loff_t)pnum * ubi->peb_size + ubi->vid_hdr_aloffset;
	err = ubi->mtd->write(ubi->mtd, addr, 4, &written, (void *)&data);
	if (!err) {
		addr -= ubi->vid_hdr_aloffset;
		err = ubi->mtd->write(ubi->mtd, addr, 4, &written,
				      (void *)&data);
		if (!err)
			return 0;
	}

	/*
	 * We failed to write to the media. This was observed with Spansion
	 * S29GL512N NOR flash. Most probably the eraseblock erasure was
	 * interrupted at a very inappropriate moment, so it became unwritable.
	 * In this case we probably anyway have garbage in this PEB.
	 */
	err1 = ubi_io_read_vid_hdr(ubi, pnum, &vid_hdr, 0);
	if (err1 == UBI_IO_BAD_VID_HDR)
		/*
		 * The VID header is corrupted, so we can safely erase this
		 * PEB and not afraid that it will be treated as a valid PEB in
		 * case of an unclean reboot.
		 */
		return 0;

	/*
	 * The PEB contains a valid VID header, but we cannot invalidate it.
	 * Supposedly the flash media or the driver is screwed up, so return an
	 * error.
	 */
	ubi_err("cannot invalidate PEB %d, write returned %d read returned %d",
		pnum, err, err1);
	ubi_dbg_dump_flash(ubi, pnum, 0, ubi->peb_size);
	return -EIO;
}

int ubi_io_sync_erase(struct ubi_device *ubi, int pnum, int torture)
{
	int err, ret = 0;

	ubi_assert(pnum >= 0 && pnum < ubi->peb_count);

	err = paranoid_check_not_bad(ubi, pnum);
	if (err != 0)
		return err;

	if (ubi->ro_mode) {
		ubi_err("read-only mode");
		return -EROFS;
	}

	if (ubi->nor_flash) {
		err = nor_erase_prepare(ubi, pnum);
		if (err)
			return err;
	}

	if (torture) {
		ret = torture_peb(ubi, pnum);
		if (ret < 0)
			return ret;
	}

	err = do_sync_erase(ubi, pnum);
	if (err)
		return err;

	return ret + 1;
}

int ubi_io_is_bad(const struct ubi_device *ubi, int pnum)
{
	struct mtd_info *mtd = ubi->mtd;

	ubi_assert(pnum >= 0 && pnum < ubi->peb_count);

	if (ubi->bad_allowed) {
		int ret;

		ret = mtd->block_isbad(mtd, (loff_t)pnum * ubi->peb_size);
		if (ret < 0)
			ubi_err("error %d while checking if PEB %d is bad",
				ret, pnum);
		else if (ret)
			dbg_io("PEB %d is bad", pnum);
		return ret;
	}

	return 0;
}

int ubi_io_mark_bad(const struct ubi_device *ubi, int pnum)
{
	int err;
	struct mtd_info *mtd = ubi->mtd;

	ubi_assert(pnum >= 0 && pnum < ubi->peb_count);

	if (ubi->ro_mode) {
		ubi_err("read-only mode");
		return -EROFS;
	}

	if (!ubi->bad_allowed)
		return 0;

	err = mtd->block_markbad(mtd, (loff_t)pnum * ubi->peb_size);
	if (err)
		ubi_err("cannot mark PEB %d bad, error %d", pnum, err);
	return err;
}

static int validate_ec_hdr(const struct ubi_device *ubi,
			   const struct ubi_ec_hdr *ec_hdr)
{
	long long ec;
	int vid_hdr_offset, leb_start;

	ec = be64_to_cpu(ec_hdr->ec);
	vid_hdr_offset = be32_to_cpu(ec_hdr->vid_hdr_offset);
	leb_start = be32_to_cpu(ec_hdr->data_offset);

	if (ec_hdr->version != UBI_VERSION) {
		ubi_err("node with incompatible UBI version found: "
			"this UBI version is %d, image version is %d",
			UBI_VERSION, (int)ec_hdr->version);
		goto bad;
	}

	if (vid_hdr_offset != ubi->vid_hdr_offset) {
		ubi_err("bad VID header offset %d, expected %d",
			vid_hdr_offset, ubi->vid_hdr_offset);
		goto bad;
	}

	if (leb_start != ubi->leb_start) {
		ubi_err("bad data offset %d, expected %d",
			leb_start, ubi->leb_start);
		goto bad;
	}

	if (ec < 0 || ec > UBI_MAX_ERASECOUNTER) {
		ubi_err("bad erase counter %lld", ec);
		goto bad;
	}

	return 0;

bad:
	ubi_err("bad EC header");
	ubi_dbg_dump_ec_hdr(ec_hdr);
	ubi_dbg_dump_stack();
	return 1;
}

int ubi_io_read_ec_hdr(struct ubi_device *ubi, int pnum,
		       struct ubi_ec_hdr *ec_hdr, int verbose)
{
	int err, read_err = 0;
	uint32_t crc, magic, hdr_crc;

	dbg_io("read EC header from PEB %d", pnum);
	ubi_assert(pnum >= 0 && pnum < ubi->peb_count);

	err = ubi_io_read(ubi, ec_hdr, pnum, 0, UBI_EC_HDR_SIZE);
	if (err) {
		if (err != UBI_IO_BITFLIPS && err != -EBADMSG)
			return err;

		/*
		 * We read all the data, but either a correctable bit-flip
		 * occurred, or MTD reported about some data integrity error,
		 * like an ECC error in case of NAND. The former is harmless,
		 * the later may mean that the read data is corrupted. But we
		 * have a CRC check-sum and we will detect this. If the EC
		 * header is still OK, we just report this as there was a
		 * bit-flip.
		 */
		read_err = err;
	}

	magic = be32_to_cpu(ec_hdr->magic);
	if (magic != UBI_EC_HDR_MAGIC) {
		/*
		 * The magic field is wrong. Let's check if we have read all
		 * 0xFF. If yes, this physical eraseblock is assumed to be
		 * empty.
		 *
		 * But if there was a read error, we do not test it for all
		 * 0xFFs. Even if it does contain all 0xFFs, this error
		 * indicates that something is still wrong with this physical
		 * eraseblock and we anyway cannot treat it as empty.
		 */
		if (read_err != -EBADMSG &&
		    check_pattern(ec_hdr, 0xFF, UBI_EC_HDR_SIZE)) {
			/* The physical eraseblock is supposedly empty */
			if (verbose)
				ubi_warn("no EC header found at PEB %d, "
					 "only 0xFF bytes", pnum);
			else if (UBI_IO_DEBUG)
				dbg_msg("no EC header found at PEB %d, "
					"only 0xFF bytes", pnum);
			return UBI_IO_PEB_EMPTY;
		}

		/*
		 * This is not a valid erase counter header, and these are not
		 * 0xFF bytes. Report that the header is corrupted.
		 */
		if (verbose) {
			ubi_warn("bad magic number at PEB %d: %08x instead of "
				 "%08x", pnum, magic, UBI_EC_HDR_MAGIC);
			ubi_dbg_dump_ec_hdr(ec_hdr);
		} else if (UBI_IO_DEBUG)
			dbg_msg("bad magic number at PEB %d: %08x instead of "
				"%08x", pnum, magic, UBI_EC_HDR_MAGIC);
		return UBI_IO_BAD_EC_HDR;
	}

	crc = crc32(UBI_CRC32_INIT, ec_hdr, UBI_EC_HDR_SIZE_CRC);
	hdr_crc = be32_to_cpu(ec_hdr->hdr_crc);

	if (hdr_crc != crc) {
		if (verbose) {
			ubi_warn("bad EC header CRC at PEB %d, calculated "
				 "%#08x, read %#08x", pnum, crc, hdr_crc);
			ubi_dbg_dump_ec_hdr(ec_hdr);
		} else if (UBI_IO_DEBUG)
			dbg_msg("bad EC header CRC at PEB %d, calculated "
				"%#08x, read %#08x", pnum, crc, hdr_crc);
		return UBI_IO_BAD_EC_HDR;
	}

	/* And of course validate what has just been read from the media */
	err = validate_ec_hdr(ubi, ec_hdr);
	if (err) {
		ubi_err("validation failed for PEB %d", pnum);
		return -EINVAL;
	}

	return read_err ? UBI_IO_BITFLIPS : 0;
}

int ubi_io_write_ec_hdr(struct ubi_device *ubi, int pnum,
			struct ubi_ec_hdr *ec_hdr)
{
	int err;
	uint32_t crc;

	dbg_io("write EC header to PEB %d", pnum);
	ubi_assert(pnum >= 0 &&  pnum < ubi->peb_count);

	ec_hdr->magic = cpu_to_be32(UBI_EC_HDR_MAGIC);
	ec_hdr->version = UBI_VERSION;
	ec_hdr->vid_hdr_offset = cpu_to_be32(ubi->vid_hdr_offset);
	ec_hdr->data_offset = cpu_to_be32(ubi->leb_start);
	ec_hdr->image_seq = cpu_to_be32(ubi->image_seq);
	crc = crc32(UBI_CRC32_INIT, ec_hdr, UBI_EC_HDR_SIZE_CRC);
	ec_hdr->hdr_crc = cpu_to_be32(crc);

	err = paranoid_check_ec_hdr(ubi, pnum, ec_hdr);
	if (err)
		return err;

	err = ubi_io_write(ubi, ec_hdr, pnum, 0, ubi->ec_hdr_alsize);
	return err;
}

static int validate_vid_hdr(const struct ubi_device *ubi,
			    const struct ubi_vid_hdr *vid_hdr)
{
	int vol_type = vid_hdr->vol_type;
	int copy_flag = vid_hdr->copy_flag;
	int vol_id = be32_to_cpu(vid_hdr->vol_id);
	int lnum = be32_to_cpu(vid_hdr->lnum);
	int compat = vid_hdr->compat;
	int data_size = be32_to_cpu(vid_hdr->data_size);
	int used_ebs = be32_to_cpu(vid_hdr->used_ebs);
	int data_pad = be32_to_cpu(vid_hdr->data_pad);
	int data_crc = be32_to_cpu(vid_hdr->data_crc);
	int usable_leb_size = ubi->leb_size - data_pad;

	if (copy_flag != 0 && copy_flag != 1) {
		dbg_err("bad copy_flag");
		goto bad;
	}

	if (vol_id < 0 || lnum < 0 || data_size < 0 || used_ebs < 0 ||
	    data_pad < 0) {
		dbg_err("negative values");
		goto bad;
	}

	if (vol_id >= UBI_MAX_VOLUMES && vol_id < UBI_INTERNAL_VOL_START) {
		dbg_err("bad vol_id");
		goto bad;
	}

	if (vol_id < UBI_INTERNAL_VOL_START && compat != 0) {
		dbg_err("bad compat");
		goto bad;
	}

	if (vol_id >= UBI_INTERNAL_VOL_START && compat != UBI_COMPAT_DELETE &&
	    compat != UBI_COMPAT_RO && compat != UBI_COMPAT_PRESERVE &&
	    compat != UBI_COMPAT_REJECT) {
		dbg_err("bad compat");
		goto bad;
	}

	if (vol_type != UBI_VID_DYNAMIC && vol_type != UBI_VID_STATIC) {
		dbg_err("bad vol_type");
		goto bad;
	}

	if (data_pad >= ubi->leb_size / 2) {
		dbg_err("bad data_pad");
		goto bad;
	}

	if (vol_type == UBI_VID_STATIC) {
		/*
		 * Although from high-level point of view static volumes may
		 * contain zero bytes of data, but no VID headers can contain
		 * zero at these fields, because they empty volumes do not have
		 * mapped logical eraseblocks.
		 */
		if (used_ebs == 0) {
			dbg_err("zero used_ebs");
			goto bad;
		}
		if (data_size == 0) {
			dbg_err("zero data_size");
			goto bad;
		}
		if (lnum < used_ebs - 1) {
			if (data_size != usable_leb_size) {
				dbg_err("bad data_size");
				goto bad;
			}
		} else if (lnum == used_ebs - 1) {
			if (data_size == 0) {
				dbg_err("bad data_size at last LEB");
				goto bad;
			}
		} else {
			dbg_err("too high lnum");
			goto bad;
		}
	} else {
		if (copy_flag == 0) {
			if (data_crc != 0) {
				dbg_err("non-zero data CRC");
				goto bad;
			}
			if (data_size != 0) {
				dbg_err("non-zero data_size");
				goto bad;
			}
		} else {
			if (data_size == 0) {
				dbg_err("zero data_size of copy");
				goto bad;
			}
		}
		if (used_ebs != 0) {
			dbg_err("bad used_ebs");
			goto bad;
		}
	}

	return 0;

bad:
	ubi_err("bad VID header");
	ubi_dbg_dump_vid_hdr(vid_hdr);
	ubi_dbg_dump_stack();
	return 1;
}

int ubi_io_read_vid_hdr(struct ubi_device *ubi, int pnum,
			struct ubi_vid_hdr *vid_hdr, int verbose)
{
	int err, read_err = 0;
	uint32_t crc, magic, hdr_crc;
	void *p;

	dbg_io("read VID header from PEB %d", pnum);
	ubi_assert(pnum >= 0 &&  pnum < ubi->peb_count);

	p = (char *)vid_hdr - ubi->vid_hdr_shift;
	err = ubi_io_read(ubi, p, pnum, ubi->vid_hdr_aloffset,
			  ubi->vid_hdr_alsize);
	if (err) {
		if (err != UBI_IO_BITFLIPS && err != -EBADMSG)
			return err;

		/*
		 * We read all the data, but either a correctable bit-flip
		 * occurred, or MTD reported about some data integrity error,
		 * like an ECC error in case of NAND. The former is harmless,
		 * the later may mean the read data is corrupted. But we have a
		 * CRC check-sum and we will identify this. If the VID header is
		 * still OK, we just report this as there was a bit-flip.
		 */
		read_err = err;
	}

	magic = be32_to_cpu(vid_hdr->magic);
	if (magic != UBI_VID_HDR_MAGIC) {
		/*
		 * If we have read all 0xFF bytes, the VID header probably does
		 * not exist and the physical eraseblock is assumed to be free.
		 *
		 * But if there was a read error, we do not test the data for
		 * 0xFFs. Even if it does contain all 0xFFs, this error
		 * indicates that something is still wrong with this physical
		 * eraseblock and it cannot be regarded as free.
		 */
		if (read_err != -EBADMSG &&
		    check_pattern(vid_hdr, 0xFF, UBI_VID_HDR_SIZE)) {
			/* The physical eraseblock is supposedly free */
			if (verbose)
				ubi_warn("no VID header found at PEB %d, "
					 "only 0xFF bytes", pnum);
			else if (UBI_IO_DEBUG)
				dbg_msg("no VID header found at PEB %d, "
					"only 0xFF bytes", pnum);
			return UBI_IO_PEB_FREE;
		}

		/*
		 * This is not a valid VID header, and these are not 0xFF
		 * bytes. Report that the header is corrupted.
		 */
		if (verbose) {
			ubi_warn("bad magic number at PEB %d: %08x instead of "
				 "%08x", pnum, magic, UBI_VID_HDR_MAGIC);
			ubi_dbg_dump_vid_hdr(vid_hdr);
		} else if (UBI_IO_DEBUG)
			dbg_msg("bad magic number at PEB %d: %08x instead of "
				"%08x", pnum, magic, UBI_VID_HDR_MAGIC);
		return UBI_IO_BAD_VID_HDR;
	}

	crc = crc32(UBI_CRC32_INIT, vid_hdr, UBI_VID_HDR_SIZE_CRC);
	hdr_crc = be32_to_cpu(vid_hdr->hdr_crc);

	if (hdr_crc != crc) {
		if (verbose) {
			ubi_warn("bad CRC at PEB %d, calculated %#08x, "
				 "read %#08x", pnum, crc, hdr_crc);
			ubi_dbg_dump_vid_hdr(vid_hdr);
		} else if (UBI_IO_DEBUG)
			dbg_msg("bad CRC at PEB %d, calculated %#08x, "
				"read %#08x", pnum, crc, hdr_crc);
		return UBI_IO_BAD_VID_HDR;
	}

	/* Validate the VID header that we have just read */
	err = validate_vid_hdr(ubi, vid_hdr);
	if (err) {
		ubi_err("validation failed for PEB %d", pnum);
		return -EINVAL;
	}

	return read_err ? UBI_IO_BITFLIPS : 0;
}

int ubi_io_write_vid_hdr(struct ubi_device *ubi, int pnum,
			 struct ubi_vid_hdr *vid_hdr)
{
	int err;
	uint32_t crc;
	void *p;

	dbg_io("write VID header to PEB %d", pnum);
	ubi_assert(pnum >= 0 &&  pnum < ubi->peb_count);

	err = paranoid_check_peb_ec_hdr(ubi, pnum);
	if (err)
		return err;

	vid_hdr->magic = cpu_to_be32(UBI_VID_HDR_MAGIC);
	vid_hdr->version = UBI_VERSION;
	crc = crc32(UBI_CRC32_INIT, vid_hdr, UBI_VID_HDR_SIZE_CRC);
	vid_hdr->hdr_crc = cpu_to_be32(crc);

	err = paranoid_check_vid_hdr(ubi, pnum, vid_hdr);
	if (err)
		return err;

	p = (char *)vid_hdr - ubi->vid_hdr_shift;
	err = ubi_io_write(ubi, p, pnum, ubi->vid_hdr_aloffset,
			   ubi->vid_hdr_alsize);
	return err;
}

#ifdef CONFIG_MTD_UBI_DEBUG_PARANOID

static int paranoid_check_not_bad(const struct ubi_device *ubi, int pnum)
{
	int err;

	err = ubi_io_is_bad(ubi, pnum);
	if (!err)
		return err;

	ubi_err("paranoid check failed for PEB %d", pnum);
	ubi_dbg_dump_stack();
	return err > 0 ? -EINVAL : err;
}

static int paranoid_check_ec_hdr(const struct ubi_device *ubi, int pnum,
				 const struct ubi_ec_hdr *ec_hdr)
{
	int err;
	uint32_t magic;

	magic = be32_to_cpu(ec_hdr->magic);
	if (magic != UBI_EC_HDR_MAGIC) {
		ubi_err("bad magic %#08x, must be %#08x",
			magic, UBI_EC_HDR_MAGIC);
		goto fail;
	}

	err = validate_ec_hdr(ubi, ec_hdr);
	if (err) {
		ubi_err("paranoid check failed for PEB %d", pnum);
		goto fail;
	}

	return 0;

fail:
	ubi_dbg_dump_ec_hdr(ec_hdr);
	ubi_dbg_dump_stack();
	return -EINVAL;
}

static int paranoid_check_peb_ec_hdr(const struct ubi_device *ubi, int pnum)
{
	int err;
	uint32_t crc, hdr_crc;
	struct ubi_ec_hdr *ec_hdr;

	ec_hdr = kzalloc(ubi->ec_hdr_alsize, GFP_NOFS);
	if (!ec_hdr)
		return -ENOMEM;

	err = ubi_io_read(ubi, ec_hdr, pnum, 0, UBI_EC_HDR_SIZE);
	if (err && err != UBI_IO_BITFLIPS && err != -EBADMSG)
		goto exit;

	crc = crc32(UBI_CRC32_INIT, ec_hdr, UBI_EC_HDR_SIZE_CRC);
	hdr_crc = be32_to_cpu(ec_hdr->hdr_crc);
	if (hdr_crc != crc) {
		ubi_err("bad CRC, calculated %#08x, read %#08x", crc, hdr_crc);
		ubi_err("paranoid check failed for PEB %d", pnum);
		ubi_dbg_dump_ec_hdr(ec_hdr);
		ubi_dbg_dump_stack();
		err = -EINVAL;
		goto exit;
	}

	err = paranoid_check_ec_hdr(ubi, pnum, ec_hdr);

exit:
	kfree(ec_hdr);
	return err;
}

static int paranoid_check_vid_hdr(const struct ubi_device *ubi, int pnum,
				  const struct ubi_vid_hdr *vid_hdr)
{
	int err;
	uint32_t magic;

	magic = be32_to_cpu(vid_hdr->magic);
	if (magic != UBI_VID_HDR_MAGIC) {
		ubi_err("bad VID header magic %#08x at PEB %d, must be %#08x",
			magic, pnum, UBI_VID_HDR_MAGIC);
		goto fail;
	}

	err = validate_vid_hdr(ubi, vid_hdr);
	if (err) {
		ubi_err("paranoid check failed for PEB %d", pnum);
		goto fail;
	}

	return err;

fail:
	ubi_err("paranoid check failed for PEB %d", pnum);
	ubi_dbg_dump_vid_hdr(vid_hdr);
	ubi_dbg_dump_stack();
	return -EINVAL;

}

static int paranoid_check_peb_vid_hdr(const struct ubi_device *ubi, int pnum)
{
	int err;
	uint32_t crc, hdr_crc;
	struct ubi_vid_hdr *vid_hdr;
	void *p;

	vid_hdr = ubi_zalloc_vid_hdr(ubi, GFP_NOFS);
	if (!vid_hdr)
		return -ENOMEM;

	p = (char *)vid_hdr - ubi->vid_hdr_shift;
	err = ubi_io_read(ubi, p, pnum, ubi->vid_hdr_aloffset,
			  ubi->vid_hdr_alsize);
	if (err && err != UBI_IO_BITFLIPS && err != -EBADMSG)
		goto exit;

	crc = crc32(UBI_CRC32_INIT, vid_hdr, UBI_EC_HDR_SIZE_CRC);
	hdr_crc = be32_to_cpu(vid_hdr->hdr_crc);
	if (hdr_crc != crc) {
		ubi_err("bad VID header CRC at PEB %d, calculated %#08x, "
			"read %#08x", pnum, crc, hdr_crc);
		ubi_err("paranoid check failed for PEB %d", pnum);
		ubi_dbg_dump_vid_hdr(vid_hdr);
		ubi_dbg_dump_stack();
		err = -EINVAL;
		goto exit;
	}

	err = paranoid_check_vid_hdr(ubi, pnum, vid_hdr);

exit:
	ubi_free_vid_hdr(ubi, vid_hdr);
	return err;
}

int ubi_dbg_check_write(struct ubi_device *ubi, const void *buf, int pnum,
			int offset, int len)
{
	int err, i;

	mutex_lock(&ubi->dbg_buf_mutex);
	err = ubi_io_read(ubi, ubi->dbg_peb_buf, pnum, offset, len);
	if (err)
		goto out_unlock;

	for (i = 0; i < len; i++) {
		uint8_t c = ((uint8_t *)buf)[i];
		uint8_t c1 = ((uint8_t *)ubi->dbg_peb_buf)[i];
		int dump_len;

		if (c == c1)
			continue;

		ubi_err("paranoid check failed for PEB %d:%d, len %d",
			pnum, offset, len);
		ubi_msg("data differ at position %d", i);
		dump_len = max_t(int, 128, len - i);
		ubi_msg("hex dump of the original buffer from %d to %d",
			i, i + dump_len);
		print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 32, 1,
			       buf + i, dump_len, 1);
		ubi_msg("hex dump of the read buffer from %d to %d",
			i, i + dump_len);
		print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 32, 1,
			       ubi->dbg_peb_buf + i, dump_len, 1);
		ubi_dbg_dump_stack();
		err = -EINVAL;
		goto out_unlock;
	}
	mutex_unlock(&ubi->dbg_buf_mutex);

	return 0;

out_unlock:
	mutex_unlock(&ubi->dbg_buf_mutex);
	return err;
}

int ubi_dbg_check_all_ff(struct ubi_device *ubi, int pnum, int offset, int len)
{
	size_t read;
	int err;
	loff_t addr = (loff_t)pnum * ubi->peb_size + offset;

	mutex_lock(&ubi->dbg_buf_mutex);
	err = ubi->mtd->read(ubi->mtd, addr, len, &read, ubi->dbg_peb_buf);
	if (err && err != -EUCLEAN) {
		ubi_err("error %d while reading %d bytes from PEB %d:%d, "
			"read %zd bytes", err, len, pnum, offset, read);
		goto error;
	}

	err = check_pattern(ubi->dbg_peb_buf, 0xFF, len);
	if (err == 0) {
		ubi_err("flash region at PEB %d:%d, length %d does not "
			"contain all 0xFF bytes", pnum, offset, len);
		goto fail;
	}
	mutex_unlock(&ubi->dbg_buf_mutex);

	return 0;

fail:
	ubi_err("paranoid check failed for PEB %d", pnum);
	ubi_msg("hex dump of the %d-%d region", offset, offset + len);
	print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 32, 1,
		       ubi->dbg_peb_buf, len, 1);
	err = -EINVAL;
error:
	ubi_dbg_dump_stack();
	mutex_unlock(&ubi->dbg_buf_mutex);
	return err;
}

#endif /* CONFIG_MTD_UBI_DEBUG_PARANOID */



#include <linux/init.h>
#include <linux/delay.h>
#include "cxgb4.h"
#include "t4_regs.h"
#include "t4fw_api.h"

static int t4_wait_op_done_val(struct adapter *adapter, int reg, u32 mask,
			       int polarity, int attempts, int delay, u32 *valp)
{
	while (1) {
		u32 val = t4_read_reg(adapter, reg);

		if (!!(val & mask) == polarity) {
			if (valp)
				*valp = val;
			return 0;
		}
		if (--attempts == 0)
			return -EAGAIN;
		if (delay)
			udelay(delay);
	}
}

static inline int t4_wait_op_done(struct adapter *adapter, int reg, u32 mask,
				  int polarity, int attempts, int delay)
{
	return t4_wait_op_done_val(adapter, reg, mask, polarity, attempts,
				   delay, NULL);
}

void t4_set_reg_field(struct adapter *adapter, unsigned int addr, u32 mask,
		      u32 val)
{
	u32 v = t4_read_reg(adapter, addr) & ~mask;

	t4_write_reg(adapter, addr, v | val);
	(void) t4_read_reg(adapter, addr);      /* flush */
}

static void t4_read_indirect(struct adapter *adap, unsigned int addr_reg,
			     unsigned int data_reg, u32 *vals,
			     unsigned int nregs, unsigned int start_idx)
{
	while (nregs--) {
		t4_write_reg(adap, addr_reg, start_idx);
		*vals++ = t4_read_reg(adap, data_reg);
		start_idx++;
	}
}

#if 0
static void t4_write_indirect(struct adapter *adap, unsigned int addr_reg,
			      unsigned int data_reg, const u32 *vals,
			      unsigned int nregs, unsigned int start_idx)
{
	while (nregs--) {
		t4_write_reg(adap, addr_reg, start_idx++);
		t4_write_reg(adap, data_reg, *vals++);
	}
}
#endif

static void get_mbox_rpl(struct adapter *adap, __be64 *rpl, int nflit,
			 u32 mbox_addr)
{
	for ( ; nflit; nflit--, mbox_addr += 8)
		*rpl++ = cpu_to_be64(t4_read_reg64(adap, mbox_addr));
}

static void fw_asrt(struct adapter *adap, u32 mbox_addr)
{
	struct fw_debug_cmd asrt;

	get_mbox_rpl(adap, (__be64 *)&asrt, sizeof(asrt) / 8, mbox_addr);
	dev_alert(adap->pdev_dev,
		  "FW assertion at %.16s:%u, val0 %#x, val1 %#x\n",
		  asrt.u.assert.filename_0_7, ntohl(asrt.u.assert.line),
		  ntohl(asrt.u.assert.x), ntohl(asrt.u.assert.y));
}

static void dump_mbox(struct adapter *adap, int mbox, u32 data_reg)
{
	dev_err(adap->pdev_dev,
		"mbox %d: %llx %llx %llx %llx %llx %llx %llx %llx\n", mbox,
		(unsigned long long)t4_read_reg64(adap, data_reg),
		(unsigned long long)t4_read_reg64(adap, data_reg + 8),
		(unsigned long long)t4_read_reg64(adap, data_reg + 16),
		(unsigned long long)t4_read_reg64(adap, data_reg + 24),
		(unsigned long long)t4_read_reg64(adap, data_reg + 32),
		(unsigned long long)t4_read_reg64(adap, data_reg + 40),
		(unsigned long long)t4_read_reg64(adap, data_reg + 48),
		(unsigned long long)t4_read_reg64(adap, data_reg + 56));
}

int t4_wr_mbox_meat(struct adapter *adap, int mbox, const void *cmd, int size,
		    void *rpl, bool sleep_ok)
{
	static int delay[] = {
		1, 1, 3, 5, 10, 10, 20, 50, 100, 200
	};

	u32 v;
	u64 res;
	int i, ms, delay_idx;
	const __be64 *p = cmd;
	u32 data_reg = PF_REG(mbox, CIM_PF_MAILBOX_DATA);
	u32 ctl_reg = PF_REG(mbox, CIM_PF_MAILBOX_CTRL);

	if ((size & 15) || size > MBOX_LEN)
		return -EINVAL;

	v = MBOWNER_GET(t4_read_reg(adap, ctl_reg));
	for (i = 0; v == MBOX_OWNER_NONE && i < 3; i++)
		v = MBOWNER_GET(t4_read_reg(adap, ctl_reg));

	if (v != MBOX_OWNER_DRV)
		return v ? -EBUSY : -ETIMEDOUT;

	for (i = 0; i < size; i += 8)
		t4_write_reg64(adap, data_reg + i, be64_to_cpu(*p++));

	t4_write_reg(adap, ctl_reg, MBMSGVALID | MBOWNER(MBOX_OWNER_FW));
	t4_read_reg(adap, ctl_reg);          /* flush write */

	delay_idx = 0;
	ms = delay[0];

	for (i = 0; i < FW_CMD_MAX_TIMEOUT; i += ms) {
		if (sleep_ok) {
			ms = delay[delay_idx];  /* last element may repeat */
			if (delay_idx < ARRAY_SIZE(delay) - 1)
				delay_idx++;
			msleep(ms);
		} else
			mdelay(ms);

		v = t4_read_reg(adap, ctl_reg);
		if (MBOWNER_GET(v) == MBOX_OWNER_DRV) {
			if (!(v & MBMSGVALID)) {
				t4_write_reg(adap, ctl_reg, 0);
				continue;
			}

			res = t4_read_reg64(adap, data_reg);
			if (FW_CMD_OP_GET(res >> 32) == FW_DEBUG_CMD) {
				fw_asrt(adap, data_reg);
				res = FW_CMD_RETVAL(EIO);
			} else if (rpl)
				get_mbox_rpl(adap, rpl, size / 8, data_reg);

			if (FW_CMD_RETVAL_GET((int)res))
				dump_mbox(adap, mbox, data_reg);
			t4_write_reg(adap, ctl_reg, 0);
			return -FW_CMD_RETVAL_GET((int)res);
		}
	}

	dump_mbox(adap, mbox, data_reg);
	dev_err(adap->pdev_dev, "command %#x in mailbox %d timed out\n",
		*(const u8 *)cmd, mbox);
	return -ETIMEDOUT;
}

int t4_mc_read(struct adapter *adap, u32 addr, __be32 *data, u64 *ecc)
{
	int i;

	if (t4_read_reg(adap, MC_BIST_CMD) & START_BIST)
		return -EBUSY;
	t4_write_reg(adap, MC_BIST_CMD_ADDR, addr & ~0x3fU);
	t4_write_reg(adap, MC_BIST_CMD_LEN, 64);
	t4_write_reg(adap, MC_BIST_DATA_PATTERN, 0xc);
	t4_write_reg(adap, MC_BIST_CMD, BIST_OPCODE(1) | START_BIST |
		     BIST_CMD_GAP(1));
	i = t4_wait_op_done(adap, MC_BIST_CMD, START_BIST, 0, 10, 1);
	if (i)
		return i;

#define MC_DATA(i) MC_BIST_STATUS_REG(MC_BIST_STATUS_RDATA, i)

	for (i = 15; i >= 0; i--)
		*data++ = htonl(t4_read_reg(adap, MC_DATA(i)));
	if (ecc)
		*ecc = t4_read_reg64(adap, MC_DATA(16));
#undef MC_DATA
	return 0;
}

int t4_edc_read(struct adapter *adap, int idx, u32 addr, __be32 *data, u64 *ecc)
{
	int i;

	idx *= EDC_STRIDE;
	if (t4_read_reg(adap, EDC_BIST_CMD + idx) & START_BIST)
		return -EBUSY;
	t4_write_reg(adap, EDC_BIST_CMD_ADDR + idx, addr & ~0x3fU);
	t4_write_reg(adap, EDC_BIST_CMD_LEN + idx, 64);
	t4_write_reg(adap, EDC_BIST_DATA_PATTERN + idx, 0xc);
	t4_write_reg(adap, EDC_BIST_CMD + idx,
		     BIST_OPCODE(1) | BIST_CMD_GAP(1) | START_BIST);
	i = t4_wait_op_done(adap, EDC_BIST_CMD + idx, START_BIST, 0, 10, 1);
	if (i)
		return i;

#define EDC_DATA(i) (EDC_BIST_STATUS_REG(EDC_BIST_STATUS_RDATA, i) + idx)

	for (i = 15; i >= 0; i--)
		*data++ = htonl(t4_read_reg(adap, EDC_DATA(i)));
	if (ecc)
		*ecc = t4_read_reg64(adap, EDC_DATA(16));
#undef EDC_DATA
	return 0;
}

struct t4_vpd_hdr {
	u8  id_tag;
	u8  id_len[2];
	u8  id_data[ID_LEN];
	u8  vpdr_tag;
	u8  vpdr_len[2];
};

#define EEPROM_STAT_ADDR   0x7bfc
#define VPD_BASE           0
#define VPD_LEN            512

int t4_seeprom_wp(struct adapter *adapter, bool enable)
{
	unsigned int v = enable ? 0xc : 0;
	int ret = pci_write_vpd(adapter->pdev, EEPROM_STAT_ADDR, 4, &v);
	return ret < 0 ? ret : 0;
}

static int get_vpd_params(struct adapter *adapter, struct vpd_params *p)
{
	int i, ret;
	int ec, sn, v2;
	u8 vpd[VPD_LEN], csum;
	unsigned int vpdr_len;
	const struct t4_vpd_hdr *v;

	ret = pci_read_vpd(adapter->pdev, VPD_BASE, sizeof(vpd), vpd);
	if (ret < 0)
		return ret;

	v = (const struct t4_vpd_hdr *)vpd;
	vpdr_len = pci_vpd_lrdt_size(&v->vpdr_tag);
	if (vpdr_len + sizeof(struct t4_vpd_hdr) > VPD_LEN) {
		dev_err(adapter->pdev_dev, "bad VPD-R length %u\n", vpdr_len);
		return -EINVAL;
	}

#define FIND_VPD_KW(var, name) do { \
	var = pci_vpd_find_info_keyword(&v->id_tag, sizeof(struct t4_vpd_hdr), \
					vpdr_len, name); \
	if (var < 0) { \
		dev_err(adapter->pdev_dev, "missing VPD keyword " name "\n"); \
		return -EINVAL; \
	} \
	var += PCI_VPD_INFO_FLD_HDR_SIZE; \
} while (0)

	FIND_VPD_KW(i, "RV");
	for (csum = 0; i >= 0; i--)
		csum += vpd[i];

	if (csum) {
		dev_err(adapter->pdev_dev,
			"corrupted VPD EEPROM, actual csum %u\n", csum);
		return -EINVAL;
	}

	FIND_VPD_KW(ec, "EC");
	FIND_VPD_KW(sn, "SN");
	FIND_VPD_KW(v2, "V2");
#undef FIND_VPD_KW

	p->cclk = simple_strtoul(vpd + v2, NULL, 10);
	memcpy(p->id, v->id_data, ID_LEN);
	strim(p->id);
	memcpy(p->ec, vpd + ec, EC_LEN);
	strim(p->ec);
	i = pci_vpd_info_field_size(vpd + sn - PCI_VPD_INFO_FLD_HDR_SIZE);
	memcpy(p->sn, vpd + sn, min(i, SERNUM_LEN));
	strim(p->sn);
	return 0;
}

/* serial flash and firmware constants */
enum {
	SF_ATTEMPTS = 10,             /* max retries for SF operations */

	/* flash command opcodes */
	SF_PROG_PAGE    = 2,          /* program page */
	SF_WR_DISABLE   = 4,          /* disable writes */
	SF_RD_STATUS    = 5,          /* read status register */
	SF_WR_ENABLE    = 6,          /* enable writes */
	SF_RD_DATA_FAST = 0xb,        /* read flash */
	SF_ERASE_SECTOR = 0xd8,       /* erase sector */

	FW_START_SEC = 8,             /* first flash sector for FW */
	FW_END_SEC = 15,              /* last flash sector for FW */
	FW_IMG_START = FW_START_SEC * SF_SEC_SIZE,
	FW_MAX_SIZE = (FW_END_SEC - FW_START_SEC + 1) * SF_SEC_SIZE,
};

static int sf1_read(struct adapter *adapter, unsigned int byte_cnt, int cont,
		    int lock, u32 *valp)
{
	int ret;

	if (!byte_cnt || byte_cnt > 4)
		return -EINVAL;
	if (t4_read_reg(adapter, SF_OP) & BUSY)
		return -EBUSY;
	cont = cont ? SF_CONT : 0;
	lock = lock ? SF_LOCK : 0;
	t4_write_reg(adapter, SF_OP, lock | cont | BYTECNT(byte_cnt - 1));
	ret = t4_wait_op_done(adapter, SF_OP, BUSY, 0, SF_ATTEMPTS, 5);
	if (!ret)
		*valp = t4_read_reg(adapter, SF_DATA);
	return ret;
}

static int sf1_write(struct adapter *adapter, unsigned int byte_cnt, int cont,
		     int lock, u32 val)
{
	if (!byte_cnt || byte_cnt > 4)
		return -EINVAL;
	if (t4_read_reg(adapter, SF_OP) & BUSY)
		return -EBUSY;
	cont = cont ? SF_CONT : 0;
	lock = lock ? SF_LOCK : 0;
	t4_write_reg(adapter, SF_DATA, val);
	t4_write_reg(adapter, SF_OP, lock |
		     cont | BYTECNT(byte_cnt - 1) | OP_WR);
	return t4_wait_op_done(adapter, SF_OP, BUSY, 0, SF_ATTEMPTS, 5);
}

static int flash_wait_op(struct adapter *adapter, int attempts, int delay)
{
	int ret;
	u32 status;

	while (1) {
		if ((ret = sf1_write(adapter, 1, 1, 1, SF_RD_STATUS)) != 0 ||
		    (ret = sf1_read(adapter, 1, 0, 1, &status)) != 0)
			return ret;
		if (!(status & 1))
			return 0;
		if (--attempts == 0)
			return -EAGAIN;
		if (delay)
			msleep(delay);
	}
}

static int t4_read_flash(struct adapter *adapter, unsigned int addr,
			 unsigned int nwords, u32 *data, int byte_oriented)
{
	int ret;

	if (addr + nwords * sizeof(u32) > SF_SIZE || (addr & 3))
		return -EINVAL;

	addr = swab32(addr) | SF_RD_DATA_FAST;

	if ((ret = sf1_write(adapter, 4, 1, 0, addr)) != 0 ||
	    (ret = sf1_read(adapter, 1, 1, 0, data)) != 0)
		return ret;

	for ( ; nwords; nwords--, data++) {
		ret = sf1_read(adapter, 4, nwords > 1, nwords == 1, data);
		if (nwords == 1)
			t4_write_reg(adapter, SF_OP, 0);    /* unlock SF */
		if (ret)
			return ret;
		if (byte_oriented)
			*data = htonl(*data);
	}
	return 0;
}

static int t4_write_flash(struct adapter *adapter, unsigned int addr,
			  unsigned int n, const u8 *data)
{
	int ret;
	u32 buf[64];
	unsigned int i, c, left, val, offset = addr & 0xff;

	if (addr >= SF_SIZE || offset + n > SF_PAGE_SIZE)
		return -EINVAL;

	val = swab32(addr) | SF_PROG_PAGE;

	if ((ret = sf1_write(adapter, 1, 0, 1, SF_WR_ENABLE)) != 0 ||
	    (ret = sf1_write(adapter, 4, 1, 1, val)) != 0)
		goto unlock;

	for (left = n; left; left -= c) {
		c = min(left, 4U);
		for (val = 0, i = 0; i < c; ++i)
			val = (val << 8) + *data++;

		ret = sf1_write(adapter, c, c != left, 1, val);
		if (ret)
			goto unlock;
	}
	ret = flash_wait_op(adapter, 5, 1);
	if (ret)
		goto unlock;

	t4_write_reg(adapter, SF_OP, 0);    /* unlock SF */

	/* Read the page to verify the write succeeded */
	ret = t4_read_flash(adapter, addr & ~0xff, ARRAY_SIZE(buf), buf, 1);
	if (ret)
		return ret;

	if (memcmp(data - n, (u8 *)buf + offset, n)) {
		dev_err(adapter->pdev_dev,
			"failed to correctly write the flash page at %#x\n",
			addr);
		return -EIO;
	}
	return 0;

unlock:
	t4_write_reg(adapter, SF_OP, 0);    /* unlock SF */
	return ret;
}

static int get_fw_version(struct adapter *adapter, u32 *vers)
{
	return t4_read_flash(adapter,
			     FW_IMG_START + offsetof(struct fw_hdr, fw_ver), 1,
			     vers, 0);
}

static int get_tp_version(struct adapter *adapter, u32 *vers)
{
	return t4_read_flash(adapter, FW_IMG_START + offsetof(struct fw_hdr,
							      tp_microcode_ver),
			     1, vers, 0);
}

int t4_check_fw_version(struct adapter *adapter)
{
	u32 api_vers[2];
	int ret, major, minor, micro;

	ret = get_fw_version(adapter, &adapter->params.fw_vers);
	if (!ret)
		ret = get_tp_version(adapter, &adapter->params.tp_vers);
	if (!ret)
		ret = t4_read_flash(adapter,
			FW_IMG_START + offsetof(struct fw_hdr, intfver_nic),
			2, api_vers, 1);
	if (ret)
		return ret;

	major = FW_HDR_FW_VER_MAJOR_GET(adapter->params.fw_vers);
	minor = FW_HDR_FW_VER_MINOR_GET(adapter->params.fw_vers);
	micro = FW_HDR_FW_VER_MICRO_GET(adapter->params.fw_vers);
	memcpy(adapter->params.api_vers, api_vers,
	       sizeof(adapter->params.api_vers));

	if (major != FW_VERSION_MAJOR) {            /* major mismatch - fail */
		dev_err(adapter->pdev_dev,
			"card FW has major version %u, driver wants %u\n",
			major, FW_VERSION_MAJOR);
		return -EINVAL;
	}

	if (minor == FW_VERSION_MINOR && micro == FW_VERSION_MICRO)
		return 0;                                   /* perfect match */

	/* Minor/micro version mismatch.  Report it but often it's OK. */
	return 1;
}

static int t4_flash_erase_sectors(struct adapter *adapter, int start, int end)
{
	int ret = 0;

	while (start <= end) {
		if ((ret = sf1_write(adapter, 1, 0, 1, SF_WR_ENABLE)) != 0 ||
		    (ret = sf1_write(adapter, 4, 0, 1,
				     SF_ERASE_SECTOR | (start << 8))) != 0 ||
		    (ret = flash_wait_op(adapter, 5, 500)) != 0) {
			dev_err(adapter->pdev_dev,
				"erase of flash sector %d failed, error %d\n",
				start, ret);
			break;
		}
		start++;
	}
	t4_write_reg(adapter, SF_OP, 0);    /* unlock SF */
	return ret;
}

int t4_load_fw(struct adapter *adap, const u8 *fw_data, unsigned int size)
{
	u32 csum;
	int ret, addr;
	unsigned int i;
	u8 first_page[SF_PAGE_SIZE];
	const u32 *p = (const u32 *)fw_data;
	const struct fw_hdr *hdr = (const struct fw_hdr *)fw_data;

	if (!size) {
		dev_err(adap->pdev_dev, "FW image has no data\n");
		return -EINVAL;
	}
	if (size & 511) {
		dev_err(adap->pdev_dev,
			"FW image size not multiple of 512 bytes\n");
		return -EINVAL;
	}
	if (ntohs(hdr->len512) * 512 != size) {
		dev_err(adap->pdev_dev,
			"FW image size differs from size in FW header\n");
		return -EINVAL;
	}
	if (size > FW_MAX_SIZE) {
		dev_err(adap->pdev_dev, "FW image too large, max is %u bytes\n",
			FW_MAX_SIZE);
		return -EFBIG;
	}

	for (csum = 0, i = 0; i < size / sizeof(csum); i++)
		csum += ntohl(p[i]);

	if (csum != 0xffffffff) {
		dev_err(adap->pdev_dev,
			"corrupted firmware image, checksum %#x\n", csum);
		return -EINVAL;
	}

	i = DIV_ROUND_UP(size, SF_SEC_SIZE);        /* # of sectors spanned */
	ret = t4_flash_erase_sectors(adap, FW_START_SEC, FW_START_SEC + i - 1);
	if (ret)
		goto out;

	/*
	 * We write the correct version at the end so the driver can see a bad
	 * version if the FW write fails.  Start by writing a copy of the
	 * first page with a bad version.
	 */
	memcpy(first_page, fw_data, SF_PAGE_SIZE);
	((struct fw_hdr *)first_page)->fw_ver = htonl(0xffffffff);
	ret = t4_write_flash(adap, FW_IMG_START, SF_PAGE_SIZE, first_page);
	if (ret)
		goto out;

	addr = FW_IMG_START;
	for (size -= SF_PAGE_SIZE; size; size -= SF_PAGE_SIZE) {
		addr += SF_PAGE_SIZE;
		fw_data += SF_PAGE_SIZE;
		ret = t4_write_flash(adap, addr, SF_PAGE_SIZE, fw_data);
		if (ret)
			goto out;
	}

	ret = t4_write_flash(adap,
			     FW_IMG_START + offsetof(struct fw_hdr, fw_ver),
			     sizeof(hdr->fw_ver), (const u8 *)&hdr->fw_ver);
out:
	if (ret)
		dev_err(adap->pdev_dev, "firmware download failed, error %d\n",
			ret);
	return ret;
}

#define ADVERT_MASK (FW_PORT_CAP_SPEED_100M | FW_PORT_CAP_SPEED_1G |\
		     FW_PORT_CAP_SPEED_10G | FW_PORT_CAP_ANEG)

int t4_link_start(struct adapter *adap, unsigned int mbox, unsigned int port,
		  struct link_config *lc)
{
	struct fw_port_cmd c;
	unsigned int fc = 0, mdi = FW_PORT_MDI(FW_PORT_MDI_AUTO);

	lc->link_ok = 0;
	if (lc->requested_fc & PAUSE_RX)
		fc |= FW_PORT_CAP_FC_RX;
	if (lc->requested_fc & PAUSE_TX)
		fc |= FW_PORT_CAP_FC_TX;

	memset(&c, 0, sizeof(c));
	c.op_to_portid = htonl(FW_CMD_OP(FW_PORT_CMD) | FW_CMD_REQUEST |
			       FW_CMD_EXEC | FW_PORT_CMD_PORTID(port));
	c.action_to_len16 = htonl(FW_PORT_CMD_ACTION(FW_PORT_ACTION_L1_CFG) |
				  FW_LEN16(c));

	if (!(lc->supported & FW_PORT_CAP_ANEG)) {
		c.u.l1cfg.rcap = htonl((lc->supported & ADVERT_MASK) | fc);
		lc->fc = lc->requested_fc & (PAUSE_RX | PAUSE_TX);
	} else if (lc->autoneg == AUTONEG_DISABLE) {
		c.u.l1cfg.rcap = htonl(lc->requested_speed | fc | mdi);
		lc->fc = lc->requested_fc & (PAUSE_RX | PAUSE_TX);
	} else
		c.u.l1cfg.rcap = htonl(lc->advertising | fc | mdi);

	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_restart_aneg(struct adapter *adap, unsigned int mbox, unsigned int port)
{
	struct fw_port_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_portid = htonl(FW_CMD_OP(FW_PORT_CMD) | FW_CMD_REQUEST |
			       FW_CMD_EXEC | FW_PORT_CMD_PORTID(port));
	c.action_to_len16 = htonl(FW_PORT_CMD_ACTION(FW_PORT_ACTION_L1_CFG) |
				  FW_LEN16(c));
	c.u.l1cfg.rcap = htonl(FW_PORT_CAP_ANEG);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

struct intr_info {
	unsigned int mask;       /* bits to check in interrupt status */
	const char *msg;         /* message to print or NULL */
	short stat_idx;          /* stat counter to increment or -1 */
	unsigned short fatal;    /* whether the condition reported is fatal */
};

static int t4_handle_intr_status(struct adapter *adapter, unsigned int reg,
				 const struct intr_info *acts)
{
	int fatal = 0;
	unsigned int mask = 0;
	unsigned int status = t4_read_reg(adapter, reg);

	for ( ; acts->mask; ++acts) {
		if (!(status & acts->mask))
			continue;
		if (acts->fatal) {
			fatal++;
			dev_alert(adapter->pdev_dev, "%s (0x%x)\n", acts->msg,
				  status & acts->mask);
		} else if (acts->msg && printk_ratelimit())
			dev_warn(adapter->pdev_dev, "%s (0x%x)\n", acts->msg,
				 status & acts->mask);
		mask |= acts->mask;
	}
	status &= mask;
	if (status)                           /* clear processed interrupts */
		t4_write_reg(adapter, reg, status);
	return fatal;
}

static void pcie_intr_handler(struct adapter *adapter)
{
	static struct intr_info sysbus_intr_info[] = {
		{ RNPP, "RXNP array parity error", -1, 1 },
		{ RPCP, "RXPC array parity error", -1, 1 },
		{ RCIP, "RXCIF array parity error", -1, 1 },
		{ RCCP, "Rx completions control array parity error", -1, 1 },
		{ RFTP, "RXFT array parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info pcie_port_intr_info[] = {
		{ TPCP, "TXPC array parity error", -1, 1 },
		{ TNPP, "TXNP array parity error", -1, 1 },
		{ TFTP, "TXFT array parity error", -1, 1 },
		{ TCAP, "TXCA array parity error", -1, 1 },
		{ TCIP, "TXCIF array parity error", -1, 1 },
		{ RCAP, "RXCA array parity error", -1, 1 },
		{ OTDD, "outbound request TLP discarded", -1, 1 },
		{ RDPE, "Rx data parity error", -1, 1 },
		{ TDUE, "Tx uncorrectable data error", -1, 1 },
		{ 0 }
	};
	static struct intr_info pcie_intr_info[] = {
		{ MSIADDRLPERR, "MSI AddrL parity error", -1, 1 },
		{ MSIADDRHPERR, "MSI AddrH parity error", -1, 1 },
		{ MSIDATAPERR, "MSI data parity error", -1, 1 },
		{ MSIXADDRLPERR, "MSI-X AddrL parity error", -1, 1 },
		{ MSIXADDRHPERR, "MSI-X AddrH parity error", -1, 1 },
		{ MSIXDATAPERR, "MSI-X data parity error", -1, 1 },
		{ MSIXDIPERR, "MSI-X DI parity error", -1, 1 },
		{ PIOCPLPERR, "PCI PIO completion FIFO parity error", -1, 1 },
		{ PIOREQPERR, "PCI PIO request FIFO parity error", -1, 1 },
		{ TARTAGPERR, "PCI PCI target tag FIFO parity error", -1, 1 },
		{ CCNTPERR, "PCI CMD channel count parity error", -1, 1 },
		{ CREQPERR, "PCI CMD channel request parity error", -1, 1 },
		{ CRSPPERR, "PCI CMD channel response parity error", -1, 1 },
		{ DCNTPERR, "PCI DMA channel count parity error", -1, 1 },
		{ DREQPERR, "PCI DMA channel request parity error", -1, 1 },
		{ DRSPPERR, "PCI DMA channel response parity error", -1, 1 },
		{ HCNTPERR, "PCI HMA channel count parity error", -1, 1 },
		{ HREQPERR, "PCI HMA channel request parity error", -1, 1 },
		{ HRSPPERR, "PCI HMA channel response parity error", -1, 1 },
		{ CFGSNPPERR, "PCI config snoop FIFO parity error", -1, 1 },
		{ FIDPERR, "PCI FID parity error", -1, 1 },
		{ INTXCLRPERR, "PCI INTx clear parity error", -1, 1 },
		{ MATAGPERR, "PCI MA tag parity error", -1, 1 },
		{ PIOTAGPERR, "PCI PIO tag parity error", -1, 1 },
		{ RXCPLPERR, "PCI Rx completion parity error", -1, 1 },
		{ RXWRPERR, "PCI Rx write parity error", -1, 1 },
		{ RPLPERR, "PCI replay buffer parity error", -1, 1 },
		{ PCIESINT, "PCI core secondary fault", -1, 1 },
		{ PCIEPINT, "PCI core primary fault", -1, 1 },
		{ UNXSPLCPLERR, "PCI unexpected split completion error", -1, 0 },
		{ 0 }
	};

	int fat;

	fat = t4_handle_intr_status(adapter,
				    PCIE_CORE_UTL_SYSTEM_BUS_AGENT_STATUS,
				    sysbus_intr_info) +
	      t4_handle_intr_status(adapter,
				    PCIE_CORE_UTL_PCI_EXPRESS_PORT_STATUS,
				    pcie_port_intr_info) +
	      t4_handle_intr_status(adapter, PCIE_INT_CAUSE, pcie_intr_info);
	if (fat)
		t4_fatal_err(adapter);
}

static void tp_intr_handler(struct adapter *adapter)
{
	static struct intr_info tp_intr_info[] = {
		{ 0x3fffffff, "TP parity error", -1, 1 },
		{ FLMTXFLSTEMPTY, "TP out of Tx pages", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adapter, TP_INT_CAUSE, tp_intr_info))
		t4_fatal_err(adapter);
}

static void sge_intr_handler(struct adapter *adapter)
{
	u64 v;

	static struct intr_info sge_intr_info[] = {
		{ ERR_CPL_EXCEED_IQE_SIZE,
		  "SGE received CPL exceeding IQE size", -1, 1 },
		{ ERR_INVALID_CIDX_INC,
		  "SGE GTS CIDX increment too large", -1, 0 },
		{ ERR_CPL_OPCODE_0, "SGE received 0-length CPL", -1, 0 },
		{ ERR_DROPPED_DB, "SGE doorbell dropped", -1, 0 },
		{ ERR_DATA_CPL_ON_HIGH_QID1 | ERR_DATA_CPL_ON_HIGH_QID0,
		  "SGE IQID > 1023 received CPL for FL", -1, 0 },
		{ ERR_BAD_DB_PIDX3, "SGE DBP 3 pidx increment too large", -1,
		  0 },
		{ ERR_BAD_DB_PIDX2, "SGE DBP 2 pidx increment too large", -1,
		  0 },
		{ ERR_BAD_DB_PIDX1, "SGE DBP 1 pidx increment too large", -1,
		  0 },
		{ ERR_BAD_DB_PIDX0, "SGE DBP 0 pidx increment too large", -1,
		  0 },
		{ ERR_ING_CTXT_PRIO,
		  "SGE too many priority ingress contexts", -1, 0 },
		{ ERR_EGR_CTXT_PRIO,
		  "SGE too many priority egress contexts", -1, 0 },
		{ INGRESS_SIZE_ERR, "SGE illegal ingress QID", -1, 0 },
		{ EGRESS_SIZE_ERR, "SGE illegal egress QID", -1, 0 },
		{ 0 }
	};

	v = (u64)t4_read_reg(adapter, SGE_INT_CAUSE1) |
	    ((u64)t4_read_reg(adapter, SGE_INT_CAUSE2) << 32);
	if (v) {
		dev_alert(adapter->pdev_dev, "SGE parity error (%#llx)\n",
			 (unsigned long long)v);
		t4_write_reg(adapter, SGE_INT_CAUSE1, v);
		t4_write_reg(adapter, SGE_INT_CAUSE2, v >> 32);
	}

	if (t4_handle_intr_status(adapter, SGE_INT_CAUSE3, sge_intr_info) ||
	    v != 0)
		t4_fatal_err(adapter);
}

static void cim_intr_handler(struct adapter *adapter)
{
	static struct intr_info cim_intr_info[] = {
		{ PREFDROPINT, "CIM control register prefetch drop", -1, 1 },
		{ OBQPARERR, "CIM OBQ parity error", -1, 1 },
		{ IBQPARERR, "CIM IBQ parity error", -1, 1 },
		{ MBUPPARERR, "CIM mailbox uP parity error", -1, 1 },
		{ MBHOSTPARERR, "CIM mailbox host parity error", -1, 1 },
		{ TIEQINPARERRINT, "CIM TIEQ outgoing parity error", -1, 1 },
		{ TIEQOUTPARERRINT, "CIM TIEQ incoming parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info cim_upintr_info[] = {
		{ RSVDSPACEINT, "CIM reserved space access", -1, 1 },
		{ ILLTRANSINT, "CIM illegal transaction", -1, 1 },
		{ ILLWRINT, "CIM illegal write", -1, 1 },
		{ ILLRDINT, "CIM illegal read", -1, 1 },
		{ ILLRDBEINT, "CIM illegal read BE", -1, 1 },
		{ ILLWRBEINT, "CIM illegal write BE", -1, 1 },
		{ SGLRDBOOTINT, "CIM single read from boot space", -1, 1 },
		{ SGLWRBOOTINT, "CIM single write to boot space", -1, 1 },
		{ BLKWRBOOTINT, "CIM block write to boot space", -1, 1 },
		{ SGLRDFLASHINT, "CIM single read from flash space", -1, 1 },
		{ SGLWRFLASHINT, "CIM single write to flash space", -1, 1 },
		{ BLKWRFLASHINT, "CIM block write to flash space", -1, 1 },
		{ SGLRDEEPROMINT, "CIM single EEPROM read", -1, 1 },
		{ SGLWREEPROMINT, "CIM single EEPROM write", -1, 1 },
		{ BLKRDEEPROMINT, "CIM block EEPROM read", -1, 1 },
		{ BLKWREEPROMINT, "CIM block EEPROM write", -1, 1 },
		{ SGLRDCTLINT , "CIM single read from CTL space", -1, 1 },
		{ SGLWRCTLINT , "CIM single write to CTL space", -1, 1 },
		{ BLKRDCTLINT , "CIM block read from CTL space", -1, 1 },
		{ BLKWRCTLINT , "CIM block write to CTL space", -1, 1 },
		{ SGLRDPLINT , "CIM single read from PL space", -1, 1 },
		{ SGLWRPLINT , "CIM single write to PL space", -1, 1 },
		{ BLKRDPLINT , "CIM block read from PL space", -1, 1 },
		{ BLKWRPLINT , "CIM block write to PL space", -1, 1 },
		{ REQOVRLOOKUPINT , "CIM request FIFO overwrite", -1, 1 },
		{ RSPOVRLOOKUPINT , "CIM response FIFO overwrite", -1, 1 },
		{ TIMEOUTINT , "CIM PIF timeout", -1, 1 },
		{ TIMEOUTMAINT , "CIM PIF MA timeout", -1, 1 },
		{ 0 }
	};

	int fat;

	fat = t4_handle_intr_status(adapter, CIM_HOST_INT_CAUSE,
				    cim_intr_info) +
	      t4_handle_intr_status(adapter, CIM_HOST_UPACC_INT_CAUSE,
				    cim_upintr_info);
	if (fat)
		t4_fatal_err(adapter);
}

static void ulprx_intr_handler(struct adapter *adapter)
{
	static struct intr_info ulprx_intr_info[] = {
		{ 0x7fffff, "ULPRX parity error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adapter, ULP_RX_INT_CAUSE, ulprx_intr_info))
		t4_fatal_err(adapter);
}

static void ulptx_intr_handler(struct adapter *adapter)
{
	static struct intr_info ulptx_intr_info[] = {
		{ PBL_BOUND_ERR_CH3, "ULPTX channel 3 PBL out of bounds", -1,
		  0 },
		{ PBL_BOUND_ERR_CH2, "ULPTX channel 2 PBL out of bounds", -1,
		  0 },
		{ PBL_BOUND_ERR_CH1, "ULPTX channel 1 PBL out of bounds", -1,
		  0 },
		{ PBL_BOUND_ERR_CH0, "ULPTX channel 0 PBL out of bounds", -1,
		  0 },
		{ 0xfffffff, "ULPTX parity error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adapter, ULP_TX_INT_CAUSE, ulptx_intr_info))
		t4_fatal_err(adapter);
}

static void pmtx_intr_handler(struct adapter *adapter)
{
	static struct intr_info pmtx_intr_info[] = {
		{ PCMD_LEN_OVFL0, "PMTX channel 0 pcmd too large", -1, 1 },
		{ PCMD_LEN_OVFL1, "PMTX channel 1 pcmd too large", -1, 1 },
		{ PCMD_LEN_OVFL2, "PMTX channel 2 pcmd too large", -1, 1 },
		{ ZERO_C_CMD_ERROR, "PMTX 0-length pcmd", -1, 1 },
		{ PMTX_FRAMING_ERROR, "PMTX framing error", -1, 1 },
		{ OESPI_PAR_ERROR, "PMTX oespi parity error", -1, 1 },
		{ DB_OPTIONS_PAR_ERROR, "PMTX db_options parity error", -1, 1 },
		{ ICSPI_PAR_ERROR, "PMTX icspi parity error", -1, 1 },
		{ C_PCMD_PAR_ERROR, "PMTX c_pcmd parity error", -1, 1},
		{ 0 }
	};

	if (t4_handle_intr_status(adapter, PM_TX_INT_CAUSE, pmtx_intr_info))
		t4_fatal_err(adapter);
}

static void pmrx_intr_handler(struct adapter *adapter)
{
	static struct intr_info pmrx_intr_info[] = {
		{ ZERO_E_CMD_ERROR, "PMRX 0-length pcmd", -1, 1 },
		{ PMRX_FRAMING_ERROR, "PMRX framing error", -1, 1 },
		{ OCSPI_PAR_ERROR, "PMRX ocspi parity error", -1, 1 },
		{ DB_OPTIONS_PAR_ERROR, "PMRX db_options parity error", -1, 1 },
		{ IESPI_PAR_ERROR, "PMRX iespi parity error", -1, 1 },
		{ E_PCMD_PAR_ERROR, "PMRX e_pcmd parity error", -1, 1},
		{ 0 }
	};

	if (t4_handle_intr_status(adapter, PM_RX_INT_CAUSE, pmrx_intr_info))
		t4_fatal_err(adapter);
}

static void cplsw_intr_handler(struct adapter *adapter)
{
	static struct intr_info cplsw_intr_info[] = {
		{ CIM_OP_MAP_PERR, "CPLSW CIM op_map parity error", -1, 1 },
		{ CIM_OVFL_ERROR, "CPLSW CIM overflow", -1, 1 },
		{ TP_FRAMING_ERROR, "CPLSW TP framing error", -1, 1 },
		{ SGE_FRAMING_ERROR, "CPLSW SGE framing error", -1, 1 },
		{ CIM_FRAMING_ERROR, "CPLSW CIM framing error", -1, 1 },
		{ ZERO_SWITCH_ERROR, "CPLSW no-switch error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adapter, CPL_INTR_CAUSE, cplsw_intr_info))
		t4_fatal_err(adapter);
}

static void le_intr_handler(struct adapter *adap)
{
	static struct intr_info le_intr_info[] = {
		{ LIPMISS, "LE LIP miss", -1, 0 },
		{ LIP0, "LE 0 LIP error", -1, 0 },
		{ PARITYERR, "LE parity error", -1, 1 },
		{ UNKNOWNCMD, "LE unknown command", -1, 1 },
		{ REQQPARERR, "LE request queue parity error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adap, LE_DB_INT_CAUSE, le_intr_info))
		t4_fatal_err(adap);
}

static void mps_intr_handler(struct adapter *adapter)
{
	static struct intr_info mps_rx_intr_info[] = {
		{ 0xffffff, "MPS Rx parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info mps_tx_intr_info[] = {
		{ TPFIFO, "MPS Tx TP FIFO parity error", -1, 1 },
		{ NCSIFIFO, "MPS Tx NC-SI FIFO parity error", -1, 1 },
		{ TXDATAFIFO, "MPS Tx data FIFO parity error", -1, 1 },
		{ TXDESCFIFO, "MPS Tx desc FIFO parity error", -1, 1 },
		{ BUBBLE, "MPS Tx underflow", -1, 1 },
		{ SECNTERR, "MPS Tx SOP/EOP error", -1, 1 },
		{ FRMERR, "MPS Tx framing error", -1, 1 },
		{ 0 }
	};
	static struct intr_info mps_trc_intr_info[] = {
		{ FILTMEM, "MPS TRC filter parity error", -1, 1 },
		{ PKTFIFO, "MPS TRC packet FIFO parity error", -1, 1 },
		{ MISCPERR, "MPS TRC misc parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info mps_stat_sram_intr_info[] = {
		{ 0x1fffff, "MPS statistics SRAM parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info mps_stat_tx_intr_info[] = {
		{ 0xfffff, "MPS statistics Tx FIFO parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info mps_stat_rx_intr_info[] = {
		{ 0xffffff, "MPS statistics Rx FIFO parity error", -1, 1 },
		{ 0 }
	};
	static struct intr_info mps_cls_intr_info[] = {
		{ MATCHSRAM, "MPS match SRAM parity error", -1, 1 },
		{ MATCHTCAM, "MPS match TCAM parity error", -1, 1 },
		{ HASHSRAM, "MPS hash SRAM parity error", -1, 1 },
		{ 0 }
	};

	int fat;

	fat = t4_handle_intr_status(adapter, MPS_RX_PERR_INT_CAUSE,
				    mps_rx_intr_info) +
	      t4_handle_intr_status(adapter, MPS_TX_INT_CAUSE,
				    mps_tx_intr_info) +
	      t4_handle_intr_status(adapter, MPS_TRC_INT_CAUSE,
				    mps_trc_intr_info) +
	      t4_handle_intr_status(adapter, MPS_STAT_PERR_INT_CAUSE_SRAM,
				    mps_stat_sram_intr_info) +
	      t4_handle_intr_status(adapter, MPS_STAT_PERR_INT_CAUSE_TX_FIFO,
				    mps_stat_tx_intr_info) +
	      t4_handle_intr_status(adapter, MPS_STAT_PERR_INT_CAUSE_RX_FIFO,
				    mps_stat_rx_intr_info) +
	      t4_handle_intr_status(adapter, MPS_CLS_INT_CAUSE,
				    mps_cls_intr_info);

	t4_write_reg(adapter, MPS_INT_CAUSE, CLSINT | TRCINT |
		     RXINT | TXINT | STATINT);
	t4_read_reg(adapter, MPS_INT_CAUSE);                    /* flush */
	if (fat)
		t4_fatal_err(adapter);
}

#define MEM_INT_MASK (PERR_INT_CAUSE | ECC_CE_INT_CAUSE | ECC_UE_INT_CAUSE)

static void mem_intr_handler(struct adapter *adapter, int idx)
{
	static const char name[3][5] = { "EDC0", "EDC1", "MC" };

	unsigned int addr, cnt_addr, v;

	if (idx <= MEM_EDC1) {
		addr = EDC_REG(EDC_INT_CAUSE, idx);
		cnt_addr = EDC_REG(EDC_ECC_STATUS, idx);
	} else {
		addr = MC_INT_CAUSE;
		cnt_addr = MC_ECC_STATUS;
	}

	v = t4_read_reg(adapter, addr) & MEM_INT_MASK;
	if (v & PERR_INT_CAUSE)
		dev_alert(adapter->pdev_dev, "%s FIFO parity error\n",
			  name[idx]);
	if (v & ECC_CE_INT_CAUSE) {
		u32 cnt = ECC_CECNT_GET(t4_read_reg(adapter, cnt_addr));

		t4_write_reg(adapter, cnt_addr, ECC_CECNT_MASK);
		if (printk_ratelimit())
			dev_warn(adapter->pdev_dev,
				 "%u %s correctable ECC data error%s\n",
				 cnt, name[idx], cnt > 1 ? "s" : "");
	}
	if (v & ECC_UE_INT_CAUSE)
		dev_alert(adapter->pdev_dev,
			  "%s uncorrectable ECC data error\n", name[idx]);

	t4_write_reg(adapter, addr, v);
	if (v & (PERR_INT_CAUSE | ECC_UE_INT_CAUSE))
		t4_fatal_err(adapter);
}

static void ma_intr_handler(struct adapter *adap)
{
	u32 v, status = t4_read_reg(adap, MA_INT_CAUSE);

	if (status & MEM_PERR_INT_CAUSE)
		dev_alert(adap->pdev_dev,
			  "MA parity error, parity status %#x\n",
			  t4_read_reg(adap, MA_PARITY_ERROR_STATUS));
	if (status & MEM_WRAP_INT_CAUSE) {
		v = t4_read_reg(adap, MA_INT_WRAP_STATUS);
		dev_alert(adap->pdev_dev, "MA address wrap-around error by "
			  "client %u to address %#x\n",
			  MEM_WRAP_CLIENT_NUM_GET(v),
			  MEM_WRAP_ADDRESS_GET(v) << 4);
	}
	t4_write_reg(adap, MA_INT_CAUSE, status);
	t4_fatal_err(adap);
}

static void smb_intr_handler(struct adapter *adap)
{
	static struct intr_info smb_intr_info[] = {
		{ MSTTXFIFOPARINT, "SMB master Tx FIFO parity error", -1, 1 },
		{ MSTRXFIFOPARINT, "SMB master Rx FIFO parity error", -1, 1 },
		{ SLVFIFOPARINT, "SMB slave FIFO parity error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adap, SMB_INT_CAUSE, smb_intr_info))
		t4_fatal_err(adap);
}

static void ncsi_intr_handler(struct adapter *adap)
{
	static struct intr_info ncsi_intr_info[] = {
		{ CIM_DM_PRTY_ERR, "NC-SI CIM parity error", -1, 1 },
		{ MPS_DM_PRTY_ERR, "NC-SI MPS parity error", -1, 1 },
		{ TXFIFO_PRTY_ERR, "NC-SI Tx FIFO parity error", -1, 1 },
		{ RXFIFO_PRTY_ERR, "NC-SI Rx FIFO parity error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adap, NCSI_INT_CAUSE, ncsi_intr_info))
		t4_fatal_err(adap);
}

static void xgmac_intr_handler(struct adapter *adap, int port)
{
	u32 v = t4_read_reg(adap, PORT_REG(port, XGMAC_PORT_INT_CAUSE));

	v &= TXFIFO_PRTY_ERR | RXFIFO_PRTY_ERR;
	if (!v)
		return;

	if (v & TXFIFO_PRTY_ERR)
		dev_alert(adap->pdev_dev, "XGMAC %d Tx FIFO parity error\n",
			  port);
	if (v & RXFIFO_PRTY_ERR)
		dev_alert(adap->pdev_dev, "XGMAC %d Rx FIFO parity error\n",
			  port);
	t4_write_reg(adap, PORT_REG(port, XGMAC_PORT_INT_CAUSE), v);
	t4_fatal_err(adap);
}

static void pl_intr_handler(struct adapter *adap)
{
	static struct intr_info pl_intr_info[] = {
		{ FATALPERR, "T4 fatal parity error", -1, 1 },
		{ PERRVFID, "PL VFID_MAP parity error", -1, 1 },
		{ 0 }
	};

	if (t4_handle_intr_status(adap, PL_PL_INT_CAUSE, pl_intr_info))
		t4_fatal_err(adap);
}

#define PF_INTR_MASK (PFSW | PFCIM)
#define GLBL_INTR_MASK (CIM | MPS | PL | PCIE | MC | EDC0 | \
		EDC1 | LE | TP | MA | PM_TX | PM_RX | ULP_RX | \
		CPL_SWITCH | SGE | ULP_TX)

int t4_slow_intr_handler(struct adapter *adapter)
{
	u32 cause = t4_read_reg(adapter, PL_INT_CAUSE);

	if (!(cause & GLBL_INTR_MASK))
		return 0;
	if (cause & CIM)
		cim_intr_handler(adapter);
	if (cause & MPS)
		mps_intr_handler(adapter);
	if (cause & NCSI)
		ncsi_intr_handler(adapter);
	if (cause & PL)
		pl_intr_handler(adapter);
	if (cause & SMB)
		smb_intr_handler(adapter);
	if (cause & XGMAC0)
		xgmac_intr_handler(adapter, 0);
	if (cause & XGMAC1)
		xgmac_intr_handler(adapter, 1);
	if (cause & XGMAC_KR0)
		xgmac_intr_handler(adapter, 2);
	if (cause & XGMAC_KR1)
		xgmac_intr_handler(adapter, 3);
	if (cause & PCIE)
		pcie_intr_handler(adapter);
	if (cause & MC)
		mem_intr_handler(adapter, MEM_MC);
	if (cause & EDC0)
		mem_intr_handler(adapter, MEM_EDC0);
	if (cause & EDC1)
		mem_intr_handler(adapter, MEM_EDC1);
	if (cause & LE)
		le_intr_handler(adapter);
	if (cause & TP)
		tp_intr_handler(adapter);
	if (cause & MA)
		ma_intr_handler(adapter);
	if (cause & PM_TX)
		pmtx_intr_handler(adapter);
	if (cause & PM_RX)
		pmrx_intr_handler(adapter);
	if (cause & ULP_RX)
		ulprx_intr_handler(adapter);
	if (cause & CPL_SWITCH)
		cplsw_intr_handler(adapter);
	if (cause & SGE)
		sge_intr_handler(adapter);
	if (cause & ULP_TX)
		ulptx_intr_handler(adapter);

	/* Clear the interrupts just processed for which we are the master. */
	t4_write_reg(adapter, PL_INT_CAUSE, cause & GLBL_INTR_MASK);
	(void) t4_read_reg(adapter, PL_INT_CAUSE); /* flush */
	return 1;
}

void t4_intr_enable(struct adapter *adapter)
{
	u32 pf = SOURCEPF_GET(t4_read_reg(adapter, PL_WHOAMI));

	t4_write_reg(adapter, SGE_INT_ENABLE3, ERR_CPL_EXCEED_IQE_SIZE |
		     ERR_INVALID_CIDX_INC | ERR_CPL_OPCODE_0 |
		     ERR_DROPPED_DB | ERR_DATA_CPL_ON_HIGH_QID1 |
		     ERR_DATA_CPL_ON_HIGH_QID0 | ERR_BAD_DB_PIDX3 |
		     ERR_BAD_DB_PIDX2 | ERR_BAD_DB_PIDX1 |
		     ERR_BAD_DB_PIDX0 | ERR_ING_CTXT_PRIO |
		     ERR_EGR_CTXT_PRIO | INGRESS_SIZE_ERR |
		     EGRESS_SIZE_ERR);
	t4_write_reg(adapter, MYPF_REG(PL_PF_INT_ENABLE), PF_INTR_MASK);
	t4_set_reg_field(adapter, PL_INT_MAP0, 0, 1 << pf);
}

void t4_intr_disable(struct adapter *adapter)
{
	u32 pf = SOURCEPF_GET(t4_read_reg(adapter, PL_WHOAMI));

	t4_write_reg(adapter, MYPF_REG(PL_PF_INT_ENABLE), 0);
	t4_set_reg_field(adapter, PL_INT_MAP0, 1 << pf, 0);
}

void t4_intr_clear(struct adapter *adapter)
{
	static const unsigned int cause_reg[] = {
		SGE_INT_CAUSE1, SGE_INT_CAUSE2, SGE_INT_CAUSE3,
		PCIE_CORE_UTL_SYSTEM_BUS_AGENT_STATUS,
		PCIE_CORE_UTL_PCI_EXPRESS_PORT_STATUS,
		PCIE_NONFAT_ERR, PCIE_INT_CAUSE,
		MC_INT_CAUSE,
		MA_INT_WRAP_STATUS, MA_PARITY_ERROR_STATUS, MA_INT_CAUSE,
		EDC_INT_CAUSE, EDC_REG(EDC_INT_CAUSE, 1),
		CIM_HOST_INT_CAUSE, CIM_HOST_UPACC_INT_CAUSE,
		MYPF_REG(CIM_PF_HOST_INT_CAUSE),
		TP_INT_CAUSE,
		ULP_RX_INT_CAUSE, ULP_TX_INT_CAUSE,
		PM_RX_INT_CAUSE, PM_TX_INT_CAUSE,
		MPS_RX_PERR_INT_CAUSE,
		CPL_INTR_CAUSE,
		MYPF_REG(PL_PF_INT_CAUSE),
		PL_PL_INT_CAUSE,
		LE_DB_INT_CAUSE,
	};

	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cause_reg); ++i)
		t4_write_reg(adapter, cause_reg[i], 0xffffffff);

	t4_write_reg(adapter, PL_INT_CAUSE, GLBL_INTR_MASK);
	(void) t4_read_reg(adapter, PL_INT_CAUSE);          /* flush */
}

static int hash_mac_addr(const u8 *addr)
{
	u32 a = ((u32)addr[0] << 16) | ((u32)addr[1] << 8) | addr[2];
	u32 b = ((u32)addr[3] << 16) | ((u32)addr[4] << 8) | addr[5];
	a ^= b;
	a ^= (a >> 12);
	a ^= (a >> 6);
	return a & 0x3f;
}

int t4_config_rss_range(struct adapter *adapter, int mbox, unsigned int viid,
			int start, int n, const u16 *rspq, unsigned int nrspq)
{
	int ret;
	const u16 *rsp = rspq;
	const u16 *rsp_end = rspq + nrspq;
	struct fw_rss_ind_tbl_cmd cmd;

	memset(&cmd, 0, sizeof(cmd));
	cmd.op_to_viid = htonl(FW_CMD_OP(FW_RSS_IND_TBL_CMD) |
			       FW_CMD_REQUEST | FW_CMD_WRITE |
			       FW_RSS_IND_TBL_CMD_VIID(viid));
	cmd.retval_len16 = htonl(FW_LEN16(cmd));

	/* each fw_rss_ind_tbl_cmd takes up to 32 entries */
	while (n > 0) {
		int nq = min(n, 32);
		__be32 *qp = &cmd.iq0_to_iq2;

		cmd.niqid = htons(nq);
		cmd.startidx = htons(start);

		start += nq;
		n -= nq;

		while (nq > 0) {
			unsigned int v;

			v = FW_RSS_IND_TBL_CMD_IQ0(*rsp);
			if (++rsp >= rsp_end)
				rsp = rspq;
			v |= FW_RSS_IND_TBL_CMD_IQ1(*rsp);
			if (++rsp >= rsp_end)
				rsp = rspq;
			v |= FW_RSS_IND_TBL_CMD_IQ2(*rsp);
			if (++rsp >= rsp_end)
				rsp = rspq;

			*qp++ = htonl(v);
			nq -= 3;
		}

		ret = t4_wr_mbox(adapter, mbox, &cmd, sizeof(cmd), NULL);
		if (ret)
			return ret;
	}
	return 0;
}

int t4_config_glbl_rss(struct adapter *adapter, int mbox, unsigned int mode,
		       unsigned int flags)
{
	struct fw_rss_glb_config_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_write = htonl(FW_CMD_OP(FW_RSS_GLB_CONFIG_CMD) |
			      FW_CMD_REQUEST | FW_CMD_WRITE);
	c.retval_len16 = htonl(FW_LEN16(c));
	if (mode == FW_RSS_GLB_CONFIG_CMD_MODE_MANUAL) {
		c.u.manual.mode_pkd = htonl(FW_RSS_GLB_CONFIG_CMD_MODE(mode));
	} else if (mode == FW_RSS_GLB_CONFIG_CMD_MODE_BASICVIRTUAL) {
		c.u.basicvirtual.mode_pkd =
			htonl(FW_RSS_GLB_CONFIG_CMD_MODE(mode));
		c.u.basicvirtual.synmapen_to_hashtoeplitz = htonl(flags);
	} else
		return -EINVAL;
	return t4_wr_mbox(adapter, mbox, &c, sizeof(c), NULL);
}

/* Read an RSS table row */
static int rd_rss_row(struct adapter *adap, int row, u32 *val)
{
	t4_write_reg(adap, TP_RSS_LKP_TABLE, 0xfff00000 | row);
	return t4_wait_op_done_val(adap, TP_RSS_LKP_TABLE, LKPTBLROWVLD, 1,
				   5, 0, val);
}

int t4_read_rss(struct adapter *adapter, u16 *map)
{
	u32 val;
	int i, ret;

	for (i = 0; i < RSS_NENTRIES / 2; ++i) {
		ret = rd_rss_row(adapter, i, &val);
		if (ret)
			return ret;
		*map++ = LKPTBLQUEUE0_GET(val);
		*map++ = LKPTBLQUEUE1_GET(val);
	}
	return 0;
}

void t4_tp_get_tcp_stats(struct adapter *adap, struct tp_tcp_stats *v4,
			 struct tp_tcp_stats *v6)
{
	u32 val[TP_MIB_TCP_RXT_SEG_LO - TP_MIB_TCP_OUT_RST + 1];

#define STAT_IDX(x) ((TP_MIB_TCP_##x) - TP_MIB_TCP_OUT_RST)
#define STAT(x)     val[STAT_IDX(x)]
#define STAT64(x)   (((u64)STAT(x##_HI) << 32) | STAT(x##_LO))

	if (v4) {
		t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, val,
				 ARRAY_SIZE(val), TP_MIB_TCP_OUT_RST);
		v4->tcpOutRsts = STAT(OUT_RST);
		v4->tcpInSegs  = STAT64(IN_SEG);
		v4->tcpOutSegs = STAT64(OUT_SEG);
		v4->tcpRetransSegs = STAT64(RXT_SEG);
	}
	if (v6) {
		t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, val,
				 ARRAY_SIZE(val), TP_MIB_TCP_V6OUT_RST);
		v6->tcpOutRsts = STAT(OUT_RST);
		v6->tcpInSegs  = STAT64(IN_SEG);
		v6->tcpOutSegs = STAT64(OUT_SEG);
		v6->tcpRetransSegs = STAT64(RXT_SEG);
	}
#undef STAT64
#undef STAT
#undef STAT_IDX
}

void t4_tp_get_err_stats(struct adapter *adap, struct tp_err_stats *st)
{
	t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, st->macInErrs,
			 12, TP_MIB_MAC_IN_ERR_0);
	t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, st->tnlCongDrops,
			 8, TP_MIB_TNL_CNG_DROP_0);
	t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, st->tnlTxDrops,
			 4, TP_MIB_TNL_DROP_0);
	t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, st->ofldVlanDrops,
			 4, TP_MIB_OFD_VLN_DROP_0);
	t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, st->tcp6InErrs,
			 4, TP_MIB_TCP_V6IN_ERR_0);
	t4_read_indirect(adap, TP_MIB_INDEX, TP_MIB_DATA, &st->ofldNoNeigh,
			 2, TP_MIB_OFD_ARP_DROP);
}

void t4_read_mtu_tbl(struct adapter *adap, u16 *mtus, u8 *mtu_log)
{
	u32 v;
	int i;

	for (i = 0; i < NMTUS; ++i) {
		t4_write_reg(adap, TP_MTU_TABLE,
			     MTUINDEX(0xff) | MTUVALUE(i));
		v = t4_read_reg(adap, TP_MTU_TABLE);
		mtus[i] = MTUVALUE_GET(v);
		if (mtu_log)
			mtu_log[i] = MTUWIDTH_GET(v);
	}
}

static void __devinit init_cong_ctrl(unsigned short *a, unsigned short *b)
{
	a[0] = a[1] = a[2] = a[3] = a[4] = a[5] = a[6] = a[7] = a[8] = 1;
	a[9] = 2;
	a[10] = 3;
	a[11] = 4;
	a[12] = 5;
	a[13] = 6;
	a[14] = 7;
	a[15] = 8;
	a[16] = 9;
	a[17] = 10;
	a[18] = 14;
	a[19] = 17;
	a[20] = 21;
	a[21] = 25;
	a[22] = 30;
	a[23] = 35;
	a[24] = 45;
	a[25] = 60;
	a[26] = 80;
	a[27] = 100;
	a[28] = 200;
	a[29] = 300;
	a[30] = 400;
	a[31] = 500;

	b[0] = b[1] = b[2] = b[3] = b[4] = b[5] = b[6] = b[7] = b[8] = 0;
	b[9] = b[10] = 1;
	b[11] = b[12] = 2;
	b[13] = b[14] = b[15] = b[16] = 3;
	b[17] = b[18] = b[19] = b[20] = b[21] = 4;
	b[22] = b[23] = b[24] = b[25] = b[26] = b[27] = 5;
	b[28] = b[29] = 6;
	b[30] = b[31] = 7;
}

/* The minimum additive increment value for the congestion control table */
#define CC_MIN_INCR 2U

void t4_load_mtus(struct adapter *adap, const unsigned short *mtus,
		  const unsigned short *alpha, const unsigned short *beta)
{
	static const unsigned int avg_pkts[NCCTRL_WIN] = {
		2, 6, 10, 14, 20, 28, 40, 56, 80, 112, 160, 224, 320, 448, 640,
		896, 1281, 1792, 2560, 3584, 5120, 7168, 10240, 14336, 20480,
		28672, 40960, 57344, 81920, 114688, 163840, 229376
	};

	unsigned int i, w;

	for (i = 0; i < NMTUS; ++i) {
		unsigned int mtu = mtus[i];
		unsigned int log2 = fls(mtu);

		if (!(mtu & ((1 << log2) >> 2)))     /* round */
			log2--;
		t4_write_reg(adap, TP_MTU_TABLE, MTUINDEX(i) |
			     MTUWIDTH(log2) | MTUVALUE(mtu));

		for (w = 0; w < NCCTRL_WIN; ++w) {
			unsigned int inc;

			inc = max(((mtu - 40) * alpha[w]) / avg_pkts[w],
				  CC_MIN_INCR);

			t4_write_reg(adap, TP_CCTRL_TABLE, (i << 21) |
				     (w << 16) | (beta[w] << 13) | inc);
		}
	}
}

int t4_set_trace_filter(struct adapter *adap, const struct trace_params *tp,
			int idx, int enable)
{
	int i, ofst = idx * 4;
	u32 data_reg, mask_reg, cfg;
	u32 multitrc = TRCMULTIFILTER;

	if (!enable) {
		t4_write_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + ofst, 0);
		goto out;
	}

	if (tp->port > 11 || tp->invert > 1 || tp->skip_len > 0x1f ||
	    tp->skip_ofst > 0x1f || tp->min_len > 0x1ff ||
	    tp->snap_len > 9600 || (idx && tp->snap_len > 256))
		return -EINVAL;

	if (tp->snap_len > 256) {            /* must be tracer 0 */
		if ((t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + 4) |
		     t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + 8) |
		     t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + 12)) & TFEN)
			return -EINVAL;  /* other tracers are enabled */
		multitrc = 0;
	} else if (idx) {
		i = t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_B);
		if (TFCAPTUREMAX_GET(i) > 256 &&
		    (t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A) & TFEN))
			return -EINVAL;
	}

	/* stop the tracer we'll be changing */
	t4_write_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + ofst, 0);

	/* disable tracing globally if running in the wrong single/multi mode */
	cfg = t4_read_reg(adap, MPS_TRC_CFG);
	if ((cfg & TRCEN) && multitrc != (cfg & TRCMULTIFILTER)) {
		t4_write_reg(adap, MPS_TRC_CFG, cfg ^ TRCEN);
		t4_read_reg(adap, MPS_TRC_CFG);                  /* flush */
		msleep(1);
		if (!(t4_read_reg(adap, MPS_TRC_CFG) & TRCFIFOEMPTY))
			return -ETIMEDOUT;
	}
	/*
	 * At this point either the tracing is enabled and in the right mode or
	 * disabled.
	 */

	idx *= (MPS_TRC_FILTER1_MATCH - MPS_TRC_FILTER0_MATCH);
	data_reg = MPS_TRC_FILTER0_MATCH + idx;
	mask_reg = MPS_TRC_FILTER0_DONT_CARE + idx;

	for (i = 0; i < TRACE_LEN / 4; i++, data_reg += 4, mask_reg += 4) {
		t4_write_reg(adap, data_reg, tp->data[i]);
		t4_write_reg(adap, mask_reg, ~tp->mask[i]);
	}
	t4_write_reg(adap, MPS_TRC_FILTER_MATCH_CTL_B + ofst,
		     TFCAPTUREMAX(tp->snap_len) |
		     TFMINPKTSIZE(tp->min_len));
	t4_write_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + ofst,
		     TFOFFSET(tp->skip_ofst) | TFLENGTH(tp->skip_len) |
		     TFPORT(tp->port) | TFEN |
		     (tp->invert ? TFINVERTMATCH : 0));

	cfg &= ~TRCMULTIFILTER;
	t4_write_reg(adap, MPS_TRC_CFG, cfg | TRCEN | multitrc);
out:	t4_read_reg(adap, MPS_TRC_CFG);  /* flush */
	return 0;
}

void t4_get_trace_filter(struct adapter *adap, struct trace_params *tp, int idx,
			 int *enabled)
{
	u32 ctla, ctlb;
	int i, ofst = idx * 4;
	u32 data_reg, mask_reg;

	ctla = t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_A + ofst);
	ctlb = t4_read_reg(adap, MPS_TRC_FILTER_MATCH_CTL_B + ofst);

	*enabled = !!(ctla & TFEN);
	tp->snap_len = TFCAPTUREMAX_GET(ctlb);
	tp->min_len = TFMINPKTSIZE_GET(ctlb);
	tp->skip_ofst = TFOFFSET_GET(ctla);
	tp->skip_len = TFLENGTH_GET(ctla);
	tp->invert = !!(ctla & TFINVERTMATCH);
	tp->port = TFPORT_GET(ctla);

	ofst = (MPS_TRC_FILTER1_MATCH - MPS_TRC_FILTER0_MATCH) * idx;
	data_reg = MPS_TRC_FILTER0_MATCH + ofst;
	mask_reg = MPS_TRC_FILTER0_DONT_CARE + ofst;

	for (i = 0; i < TRACE_LEN / 4; i++, data_reg += 4, mask_reg += 4) {
		tp->mask[i] = ~t4_read_reg(adap, mask_reg);
		tp->data[i] = t4_read_reg(adap, data_reg) & tp->mask[i];
	}
}

static unsigned int get_mps_bg_map(struct adapter *adap, int idx)
{
	u32 n = NUMPORTS_GET(t4_read_reg(adap, MPS_CMN_CTL));

	if (n == 0)
		return idx == 0 ? 0xf : 0;
	if (n == 1)
		return idx < 2 ? (3 << (2 * idx)) : 0;
	return 1 << idx;
}

void t4_get_port_stats(struct adapter *adap, int idx, struct port_stats *p)
{
	u32 bgmap = get_mps_bg_map(adap, idx);

#define GET_STAT(name) \
	t4_read_reg64(adap, PORT_REG(idx, MPS_PORT_STAT_##name##_L))
#define GET_STAT_COM(name) t4_read_reg64(adap, MPS_STAT_##name##_L)

	p->tx_octets           = GET_STAT(TX_PORT_BYTES);
	p->tx_frames           = GET_STAT(TX_PORT_FRAMES);
	p->tx_bcast_frames     = GET_STAT(TX_PORT_BCAST);
	p->tx_mcast_frames     = GET_STAT(TX_PORT_MCAST);
	p->tx_ucast_frames     = GET_STAT(TX_PORT_UCAST);
	p->tx_error_frames     = GET_STAT(TX_PORT_ERROR);
	p->tx_frames_64        = GET_STAT(TX_PORT_64B);
	p->tx_frames_65_127    = GET_STAT(TX_PORT_65B_127B);
	p->tx_frames_128_255   = GET_STAT(TX_PORT_128B_255B);
	p->tx_frames_256_511   = GET_STAT(TX_PORT_256B_511B);
	p->tx_frames_512_1023  = GET_STAT(TX_PORT_512B_1023B);
	p->tx_frames_1024_1518 = GET_STAT(TX_PORT_1024B_1518B);
	p->tx_frames_1519_max  = GET_STAT(TX_PORT_1519B_MAX);
	p->tx_drop             = GET_STAT(TX_PORT_DROP);
	p->tx_pause            = GET_STAT(TX_PORT_PAUSE);
	p->tx_ppp0             = GET_STAT(TX_PORT_PPP0);
	p->tx_ppp1             = GET_STAT(TX_PORT_PPP1);
	p->tx_ppp2             = GET_STAT(TX_PORT_PPP2);
	p->tx_ppp3             = GET_STAT(TX_PORT_PPP3);
	p->tx_ppp4             = GET_STAT(TX_PORT_PPP4);
	p->tx_ppp5             = GET_STAT(TX_PORT_PPP5);
	p->tx_ppp6             = GET_STAT(TX_PORT_PPP6);
	p->tx_ppp7             = GET_STAT(TX_PORT_PPP7);

	p->rx_octets           = GET_STAT(RX_PORT_BYTES);
	p->rx_frames           = GET_STAT(RX_PORT_FRAMES);
	p->rx_bcast_frames     = GET_STAT(RX_PORT_BCAST);
	p->rx_mcast_frames     = GET_STAT(RX_PORT_MCAST);
	p->rx_ucast_frames     = GET_STAT(RX_PORT_UCAST);
	p->rx_too_long         = GET_STAT(RX_PORT_MTU_ERROR);
	p->rx_jabber           = GET_STAT(RX_PORT_MTU_CRC_ERROR);
	p->rx_fcs_err          = GET_STAT(RX_PORT_CRC_ERROR);
	p->rx_len_err          = GET_STAT(RX_PORT_LEN_ERROR);
	p->rx_symbol_err       = GET_STAT(RX_PORT_SYM_ERROR);
	p->rx_runt             = GET_STAT(RX_PORT_LESS_64B);
	p->rx_frames_64        = GET_STAT(RX_PORT_64B);
	p->rx_frames_65_127    = GET_STAT(RX_PORT_65B_127B);
	p->rx_frames_128_255   = GET_STAT(RX_PORT_128B_255B);
	p->rx_frames_256_511   = GET_STAT(RX_PORT_256B_511B);
	p->rx_frames_512_1023  = GET_STAT(RX_PORT_512B_1023B);
	p->rx_frames_1024_1518 = GET_STAT(RX_PORT_1024B_1518B);
	p->rx_frames_1519_max  = GET_STAT(RX_PORT_1519B_MAX);
	p->rx_pause            = GET_STAT(RX_PORT_PAUSE);
	p->rx_ppp0             = GET_STAT(RX_PORT_PPP0);
	p->rx_ppp1             = GET_STAT(RX_PORT_PPP1);
	p->rx_ppp2             = GET_STAT(RX_PORT_PPP2);
	p->rx_ppp3             = GET_STAT(RX_PORT_PPP3);
	p->rx_ppp4             = GET_STAT(RX_PORT_PPP4);
	p->rx_ppp5             = GET_STAT(RX_PORT_PPP5);
	p->rx_ppp6             = GET_STAT(RX_PORT_PPP6);
	p->rx_ppp7             = GET_STAT(RX_PORT_PPP7);

	p->rx_ovflow0 = (bgmap & 1) ? GET_STAT_COM(RX_BG_0_MAC_DROP_FRAME) : 0;
	p->rx_ovflow1 = (bgmap & 2) ? GET_STAT_COM(RX_BG_1_MAC_DROP_FRAME) : 0;
	p->rx_ovflow2 = (bgmap & 4) ? GET_STAT_COM(RX_BG_2_MAC_DROP_FRAME) : 0;
	p->rx_ovflow3 = (bgmap & 8) ? GET_STAT_COM(RX_BG_3_MAC_DROP_FRAME) : 0;
	p->rx_trunc0 = (bgmap & 1) ? GET_STAT_COM(RX_BG_0_MAC_TRUNC_FRAME) : 0;
	p->rx_trunc1 = (bgmap & 2) ? GET_STAT_COM(RX_BG_1_MAC_TRUNC_FRAME) : 0;
	p->rx_trunc2 = (bgmap & 4) ? GET_STAT_COM(RX_BG_2_MAC_TRUNC_FRAME) : 0;
	p->rx_trunc3 = (bgmap & 8) ? GET_STAT_COM(RX_BG_3_MAC_TRUNC_FRAME) : 0;

#undef GET_STAT
#undef GET_STAT_COM
}

void t4_get_lb_stats(struct adapter *adap, int idx, struct lb_port_stats *p)
{
	u32 bgmap = get_mps_bg_map(adap, idx);

#define GET_STAT(name) \
	t4_read_reg64(adap, PORT_REG(idx, MPS_PORT_STAT_LB_PORT_##name##_L))
#define GET_STAT_COM(name) t4_read_reg64(adap, MPS_STAT_##name##_L)

	p->octets           = GET_STAT(BYTES);
	p->frames           = GET_STAT(FRAMES);
	p->bcast_frames     = GET_STAT(BCAST);
	p->mcast_frames     = GET_STAT(MCAST);
	p->ucast_frames     = GET_STAT(UCAST);
	p->error_frames     = GET_STAT(ERROR);

	p->frames_64        = GET_STAT(64B);
	p->frames_65_127    = GET_STAT(65B_127B);
	p->frames_128_255   = GET_STAT(128B_255B);
	p->frames_256_511   = GET_STAT(256B_511B);
	p->frames_512_1023  = GET_STAT(512B_1023B);
	p->frames_1024_1518 = GET_STAT(1024B_1518B);
	p->frames_1519_max  = GET_STAT(1519B_MAX);
	p->drop             = t4_read_reg(adap, PORT_REG(idx,
					  MPS_PORT_STAT_LB_PORT_DROP_FRAMES));

	p->ovflow0 = (bgmap & 1) ? GET_STAT_COM(RX_BG_0_LB_DROP_FRAME) : 0;
	p->ovflow1 = (bgmap & 2) ? GET_STAT_COM(RX_BG_1_LB_DROP_FRAME) : 0;
	p->ovflow2 = (bgmap & 4) ? GET_STAT_COM(RX_BG_2_LB_DROP_FRAME) : 0;
	p->ovflow3 = (bgmap & 8) ? GET_STAT_COM(RX_BG_3_LB_DROP_FRAME) : 0;
	p->trunc0 = (bgmap & 1) ? GET_STAT_COM(RX_BG_0_LB_TRUNC_FRAME) : 0;
	p->trunc1 = (bgmap & 2) ? GET_STAT_COM(RX_BG_1_LB_TRUNC_FRAME) : 0;
	p->trunc2 = (bgmap & 4) ? GET_STAT_COM(RX_BG_2_LB_TRUNC_FRAME) : 0;
	p->trunc3 = (bgmap & 8) ? GET_STAT_COM(RX_BG_3_LB_TRUNC_FRAME) : 0;

#undef GET_STAT
#undef GET_STAT_COM
}

void t4_wol_magic_enable(struct adapter *adap, unsigned int port,
			 const u8 *addr)
{
	if (addr) {
		t4_write_reg(adap, PORT_REG(port, XGMAC_PORT_MAGIC_MACID_LO),
			     (addr[2] << 24) | (addr[3] << 16) |
			     (addr[4] << 8) | addr[5]);
		t4_write_reg(adap, PORT_REG(port, XGMAC_PORT_MAGIC_MACID_HI),
			     (addr[0] << 8) | addr[1]);
	}
	t4_set_reg_field(adap, PORT_REG(port, XGMAC_PORT_CFG2), MAGICEN,
			 addr ? MAGICEN : 0);
}

int t4_wol_pat_enable(struct adapter *adap, unsigned int port, unsigned int map,
		      u64 mask0, u64 mask1, unsigned int crc, bool enable)
{
	int i;

	if (!enable) {
		t4_set_reg_field(adap, PORT_REG(port, XGMAC_PORT_CFG2),
				 PATEN, 0);
		return 0;
	}
	if (map > 0xff)
		return -EINVAL;

#define EPIO_REG(name) PORT_REG(port, XGMAC_PORT_EPIO_##name)

	t4_write_reg(adap, EPIO_REG(DATA1), mask0 >> 32);
	t4_write_reg(adap, EPIO_REG(DATA2), mask1);
	t4_write_reg(adap, EPIO_REG(DATA3), mask1 >> 32);

	for (i = 0; i < NWOL_PAT; i++, map >>= 1) {
		if (!(map & 1))
			continue;

		/* write byte masks */
		t4_write_reg(adap, EPIO_REG(DATA0), mask0);
		t4_write_reg(adap, EPIO_REG(OP), ADDRESS(i) | EPIOWR);
		t4_read_reg(adap, EPIO_REG(OP));                /* flush */
		if (t4_read_reg(adap, EPIO_REG(OP)) & BUSY)
			return -ETIMEDOUT;

		/* write CRC */
		t4_write_reg(adap, EPIO_REG(DATA0), crc);
		t4_write_reg(adap, EPIO_REG(OP), ADDRESS(i + 32) | EPIOWR);
		t4_read_reg(adap, EPIO_REG(OP));                /* flush */
		if (t4_read_reg(adap, EPIO_REG(OP)) & BUSY)
			return -ETIMEDOUT;
	}
#undef EPIO_REG

	t4_set_reg_field(adap, PORT_REG(port, XGMAC_PORT_CFG2), 0, PATEN);
	return 0;
}

#define INIT_CMD(var, cmd, rd_wr) do { \
	(var).op_to_write = htonl(FW_CMD_OP(FW_##cmd##_CMD) | \
				  FW_CMD_REQUEST | FW_CMD_##rd_wr); \
	(var).retval_len16 = htonl(FW_LEN16(var)); \
} while (0)

int t4_mdio_rd(struct adapter *adap, unsigned int mbox, unsigned int phy_addr,
	       unsigned int mmd, unsigned int reg, u16 *valp)
{
	int ret;
	struct fw_ldst_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_addrspace = htonl(FW_CMD_OP(FW_LDST_CMD) | FW_CMD_REQUEST |
		FW_CMD_READ | FW_LDST_CMD_ADDRSPACE(FW_LDST_ADDRSPC_MDIO));
	c.cycles_to_len16 = htonl(FW_LEN16(c));
	c.u.mdio.paddr_mmd = htons(FW_LDST_CMD_PADDR(phy_addr) |
				   FW_LDST_CMD_MMD(mmd));
	c.u.mdio.raddr = htons(reg);

	ret = t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
	if (ret == 0)
		*valp = ntohs(c.u.mdio.rval);
	return ret;
}

int t4_mdio_wr(struct adapter *adap, unsigned int mbox, unsigned int phy_addr,
	       unsigned int mmd, unsigned int reg, u16 val)
{
	struct fw_ldst_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_addrspace = htonl(FW_CMD_OP(FW_LDST_CMD) | FW_CMD_REQUEST |
		FW_CMD_WRITE | FW_LDST_CMD_ADDRSPACE(FW_LDST_ADDRSPC_MDIO));
	c.cycles_to_len16 = htonl(FW_LEN16(c));
	c.u.mdio.paddr_mmd = htons(FW_LDST_CMD_PADDR(phy_addr) |
				   FW_LDST_CMD_MMD(mmd));
	c.u.mdio.raddr = htons(reg);
	c.u.mdio.rval = htons(val);

	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_fw_hello(struct adapter *adap, unsigned int mbox, unsigned int evt_mbox,
		enum dev_master master, enum dev_state *state)
{
	int ret;
	struct fw_hello_cmd c;

	INIT_CMD(c, HELLO, WRITE);
	c.err_to_mbasyncnot = htonl(
		FW_HELLO_CMD_MASTERDIS(master == MASTER_CANT) |
		FW_HELLO_CMD_MASTERFORCE(master == MASTER_MUST) |
		FW_HELLO_CMD_MBMASTER(master == MASTER_MUST ? mbox : 0xff) |
		FW_HELLO_CMD_MBASYNCNOT(evt_mbox));

	ret = t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
	if (ret == 0 && state) {
		u32 v = ntohl(c.err_to_mbasyncnot);
		if (v & FW_HELLO_CMD_INIT)
			*state = DEV_STATE_INIT;
		else if (v & FW_HELLO_CMD_ERR)
			*state = DEV_STATE_ERR;
		else
			*state = DEV_STATE_UNINIT;
	}
	return ret;
}

int t4_fw_bye(struct adapter *adap, unsigned int mbox)
{
	struct fw_bye_cmd c;

	INIT_CMD(c, BYE, WRITE);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_early_init(struct adapter *adap, unsigned int mbox)
{
	struct fw_initialize_cmd c;

	INIT_CMD(c, INITIALIZE, WRITE);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_fw_reset(struct adapter *adap, unsigned int mbox, int reset)
{
	struct fw_reset_cmd c;

	INIT_CMD(c, RESET, WRITE);
	c.val = htonl(reset);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_query_params(struct adapter *adap, unsigned int mbox, unsigned int pf,
		    unsigned int vf, unsigned int nparams, const u32 *params,
		    u32 *val)
{
	int i, ret;
	struct fw_params_cmd c;
	__be32 *p = &c.param[0].mnem;

	if (nparams > 7)
		return -EINVAL;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_PARAMS_CMD) | FW_CMD_REQUEST |
			    FW_CMD_READ | FW_PARAMS_CMD_PFN(pf) |
			    FW_PARAMS_CMD_VFN(vf));
	c.retval_len16 = htonl(FW_LEN16(c));
	for (i = 0; i < nparams; i++, p += 2)
		*p = htonl(*params++);

	ret = t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
	if (ret == 0)
		for (i = 0, p = &c.param[0].val; i < nparams; i++, p += 2)
			*val++ = ntohl(*p);
	return ret;
}

int t4_set_params(struct adapter *adap, unsigned int mbox, unsigned int pf,
		  unsigned int vf, unsigned int nparams, const u32 *params,
		  const u32 *val)
{
	struct fw_params_cmd c;
	__be32 *p = &c.param[0].mnem;

	if (nparams > 7)
		return -EINVAL;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_PARAMS_CMD) | FW_CMD_REQUEST |
			    FW_CMD_WRITE | FW_PARAMS_CMD_PFN(pf) |
			    FW_PARAMS_CMD_VFN(vf));
	c.retval_len16 = htonl(FW_LEN16(c));
	while (nparams--) {
		*p++ = htonl(*params++);
		*p++ = htonl(*val++);
	}

	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_cfg_pfvf(struct adapter *adap, unsigned int mbox, unsigned int pf,
		unsigned int vf, unsigned int txq, unsigned int txq_eth_ctrl,
		unsigned int rxqi, unsigned int rxq, unsigned int tc,
		unsigned int vi, unsigned int cmask, unsigned int pmask,
		unsigned int nexact, unsigned int rcaps, unsigned int wxcaps)
{
	struct fw_pfvf_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_PFVF_CMD) | FW_CMD_REQUEST |
			    FW_CMD_WRITE | FW_PFVF_CMD_PFN(pf) |
			    FW_PFVF_CMD_VFN(vf));
	c.retval_len16 = htonl(FW_LEN16(c));
	c.niqflint_niq = htonl(FW_PFVF_CMD_NIQFLINT(rxqi) |
			       FW_PFVF_CMD_NIQ(rxq));
	c.cmask_to_neq = htonl(FW_PFVF_CMD_CMASK(cmask) |
			       FW_PFVF_CMD_PMASK(pmask) |
			       FW_PFVF_CMD_NEQ(txq));
	c.tc_to_nexactf = htonl(FW_PFVF_CMD_TC(tc) | FW_PFVF_CMD_NVI(vi) |
				FW_PFVF_CMD_NEXACTF(nexact));
	c.r_caps_to_nethctrl = htonl(FW_PFVF_CMD_R_CAPS(rcaps) |
				     FW_PFVF_CMD_WX_CAPS(wxcaps) |
				     FW_PFVF_CMD_NETHCTRL(txq_eth_ctrl));
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_alloc_vi(struct adapter *adap, unsigned int mbox, unsigned int port,
		unsigned int pf, unsigned int vf, unsigned int nmac, u8 *mac,
		unsigned int *rss_size)
{
	int ret;
	struct fw_vi_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_VI_CMD) | FW_CMD_REQUEST |
			    FW_CMD_WRITE | FW_CMD_EXEC |
			    FW_VI_CMD_PFN(pf) | FW_VI_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_VI_CMD_ALLOC | FW_LEN16(c));
	c.portid_pkd = FW_VI_CMD_PORTID(port);
	c.nmac = nmac - 1;

	ret = t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
	if (ret)
		return ret;

	if (mac) {
		memcpy(mac, c.mac, sizeof(c.mac));
		switch (nmac) {
		case 5:
			memcpy(mac + 24, c.nmac3, sizeof(c.nmac3));
		case 4:
			memcpy(mac + 18, c.nmac2, sizeof(c.nmac2));
		case 3:
			memcpy(mac + 12, c.nmac1, sizeof(c.nmac1));
		case 2:
			memcpy(mac + 6,  c.nmac0, sizeof(c.nmac0));
		}
	}
	if (rss_size)
		*rss_size = FW_VI_CMD_RSSSIZE_GET(ntohs(c.rsssize_pkd));
	return ntohs(c.viid_pkd);
}

int t4_free_vi(struct adapter *adap, unsigned int mbox, unsigned int pf,
	       unsigned int vf, unsigned int viid)
{
	struct fw_vi_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_VI_CMD) | FW_CMD_REQUEST |
			    FW_CMD_EXEC | FW_VI_CMD_PFN(pf) |
			    FW_VI_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_VI_CMD_FREE | FW_LEN16(c));
	c.viid_pkd = htons(FW_VI_CMD_VIID(viid));
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
}

int t4_set_rxmode(struct adapter *adap, unsigned int mbox, unsigned int viid,
		  int mtu, int promisc, int all_multi, int bcast, int vlanex,
		  bool sleep_ok)
{
	struct fw_vi_rxmode_cmd c;

	/* convert to FW values */
	if (mtu < 0)
		mtu = FW_RXMODE_MTU_NO_CHG;
	if (promisc < 0)
		promisc = FW_VI_RXMODE_CMD_PROMISCEN_MASK;
	if (all_multi < 0)
		all_multi = FW_VI_RXMODE_CMD_ALLMULTIEN_MASK;
	if (bcast < 0)
		bcast = FW_VI_RXMODE_CMD_BROADCASTEN_MASK;
	if (vlanex < 0)
		vlanex = FW_VI_RXMODE_CMD_VLANEXEN_MASK;

	memset(&c, 0, sizeof(c));
	c.op_to_viid = htonl(FW_CMD_OP(FW_VI_RXMODE_CMD) | FW_CMD_REQUEST |
			     FW_CMD_WRITE | FW_VI_RXMODE_CMD_VIID(viid));
	c.retval_len16 = htonl(FW_LEN16(c));
	c.mtu_to_vlanexen = htonl(FW_VI_RXMODE_CMD_MTU(mtu) |
				  FW_VI_RXMODE_CMD_PROMISCEN(promisc) |
				  FW_VI_RXMODE_CMD_ALLMULTIEN(all_multi) |
				  FW_VI_RXMODE_CMD_BROADCASTEN(bcast) |
				  FW_VI_RXMODE_CMD_VLANEXEN(vlanex));
	return t4_wr_mbox_meat(adap, mbox, &c, sizeof(c), NULL, sleep_ok);
}

int t4_alloc_mac_filt(struct adapter *adap, unsigned int mbox,
		      unsigned int viid, bool free, unsigned int naddr,
		      const u8 **addr, u16 *idx, u64 *hash, bool sleep_ok)
{
	int i, ret;
	struct fw_vi_mac_cmd c;
	struct fw_vi_mac_exact *p;

	if (naddr > 7)
		return -EINVAL;

	memset(&c, 0, sizeof(c));
	c.op_to_viid = htonl(FW_CMD_OP(FW_VI_MAC_CMD) | FW_CMD_REQUEST |
			     FW_CMD_WRITE | (free ? FW_CMD_EXEC : 0) |
			     FW_VI_MAC_CMD_VIID(viid));
	c.freemacs_to_len16 = htonl(FW_VI_MAC_CMD_FREEMACS(free) |
				    FW_CMD_LEN16((naddr + 2) / 2));

	for (i = 0, p = c.u.exact; i < naddr; i++, p++) {
		p->valid_to_idx = htons(FW_VI_MAC_CMD_VALID |
				      FW_VI_MAC_CMD_IDX(FW_VI_MAC_ADD_MAC));
		memcpy(p->macaddr, addr[i], sizeof(p->macaddr));
	}

	ret = t4_wr_mbox_meat(adap, mbox, &c, sizeof(c), &c, sleep_ok);
	if (ret)
		return ret;

	for (i = 0, p = c.u.exact; i < naddr; i++, p++) {
		u16 index = FW_VI_MAC_CMD_IDX_GET(ntohs(p->valid_to_idx));

		if (idx)
			idx[i] = index >= NEXACT_MAC ? 0xffff : index;
		if (index < NEXACT_MAC)
			ret++;
		else if (hash)
			*hash |= (1 << hash_mac_addr(addr[i]));
	}
	return ret;
}

int t4_change_mac(struct adapter *adap, unsigned int mbox, unsigned int viid,
		  int idx, const u8 *addr, bool persist, bool add_smt)
{
	int ret, mode;
	struct fw_vi_mac_cmd c;
	struct fw_vi_mac_exact *p = c.u.exact;

	if (idx < 0)                             /* new allocation */
		idx = persist ? FW_VI_MAC_ADD_PERSIST_MAC : FW_VI_MAC_ADD_MAC;
	mode = add_smt ? FW_VI_MAC_SMT_AND_MPSTCAM : FW_VI_MAC_MPS_TCAM_ENTRY;

	memset(&c, 0, sizeof(c));
	c.op_to_viid = htonl(FW_CMD_OP(FW_VI_MAC_CMD) | FW_CMD_REQUEST |
			     FW_CMD_WRITE | FW_VI_MAC_CMD_VIID(viid));
	c.freemacs_to_len16 = htonl(FW_CMD_LEN16(1));
	p->valid_to_idx = htons(FW_VI_MAC_CMD_VALID |
				FW_VI_MAC_CMD_SMAC_RESULT(mode) |
				FW_VI_MAC_CMD_IDX(idx));
	memcpy(p->macaddr, addr, sizeof(p->macaddr));

	ret = t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
	if (ret == 0) {
		ret = FW_VI_MAC_CMD_IDX_GET(ntohs(p->valid_to_idx));
		if (ret >= NEXACT_MAC)
			ret = -ENOMEM;
	}
	return ret;
}

int t4_set_addr_hash(struct adapter *adap, unsigned int mbox, unsigned int viid,
		     bool ucast, u64 vec, bool sleep_ok)
{
	struct fw_vi_mac_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_viid = htonl(FW_CMD_OP(FW_VI_MAC_CMD) | FW_CMD_REQUEST |
			     FW_CMD_WRITE | FW_VI_ENABLE_CMD_VIID(viid));
	c.freemacs_to_len16 = htonl(FW_VI_MAC_CMD_HASHVECEN |
				    FW_VI_MAC_CMD_HASHUNIEN(ucast) |
				    FW_CMD_LEN16(1));
	c.u.hash.hashvec = cpu_to_be64(vec);
	return t4_wr_mbox_meat(adap, mbox, &c, sizeof(c), NULL, sleep_ok);
}

int t4_enable_vi(struct adapter *adap, unsigned int mbox, unsigned int viid,
		 bool rx_en, bool tx_en)
{
	struct fw_vi_enable_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_viid = htonl(FW_CMD_OP(FW_VI_ENABLE_CMD) | FW_CMD_REQUEST |
			     FW_CMD_EXEC | FW_VI_ENABLE_CMD_VIID(viid));
	c.ien_to_len16 = htonl(FW_VI_ENABLE_CMD_IEN(rx_en) |
			       FW_VI_ENABLE_CMD_EEN(tx_en) | FW_LEN16(c));
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_identify_port(struct adapter *adap, unsigned int mbox, unsigned int viid,
		     unsigned int nblinks)
{
	struct fw_vi_enable_cmd c;

	c.op_to_viid = htonl(FW_CMD_OP(FW_VI_ENABLE_CMD) | FW_CMD_REQUEST |
			     FW_CMD_EXEC | FW_VI_ENABLE_CMD_VIID(viid));
	c.ien_to_len16 = htonl(FW_VI_ENABLE_CMD_LED | FW_LEN16(c));
	c.blinkdur = htons(nblinks);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_iq_start_stop(struct adapter *adap, unsigned int mbox, bool start,
		     unsigned int pf, unsigned int vf, unsigned int iqid,
		     unsigned int fl0id, unsigned int fl1id)
{
	struct fw_iq_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_IQ_CMD) | FW_CMD_REQUEST |
			    FW_CMD_EXEC | FW_IQ_CMD_PFN(pf) |
			    FW_IQ_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_IQ_CMD_IQSTART(start) |
				 FW_IQ_CMD_IQSTOP(!start) | FW_LEN16(c));
	c.iqid = htons(iqid);
	c.fl0id = htons(fl0id);
	c.fl1id = htons(fl1id);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_iq_free(struct adapter *adap, unsigned int mbox, unsigned int pf,
	       unsigned int vf, unsigned int iqtype, unsigned int iqid,
	       unsigned int fl0id, unsigned int fl1id)
{
	struct fw_iq_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_IQ_CMD) | FW_CMD_REQUEST |
			    FW_CMD_EXEC | FW_IQ_CMD_PFN(pf) |
			    FW_IQ_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_IQ_CMD_FREE | FW_LEN16(c));
	c.type_to_iqandstindex = htonl(FW_IQ_CMD_TYPE(iqtype));
	c.iqid = htons(iqid);
	c.fl0id = htons(fl0id);
	c.fl1id = htons(fl1id);
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_eth_eq_free(struct adapter *adap, unsigned int mbox, unsigned int pf,
		   unsigned int vf, unsigned int eqid)
{
	struct fw_eq_eth_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_EQ_ETH_CMD) | FW_CMD_REQUEST |
			    FW_CMD_EXEC | FW_EQ_ETH_CMD_PFN(pf) |
			    FW_EQ_ETH_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_EQ_ETH_CMD_FREE | FW_LEN16(c));
	c.eqid_pkd = htonl(FW_EQ_ETH_CMD_EQID(eqid));
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_ctrl_eq_free(struct adapter *adap, unsigned int mbox, unsigned int pf,
		    unsigned int vf, unsigned int eqid)
{
	struct fw_eq_ctrl_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_EQ_CTRL_CMD) | FW_CMD_REQUEST |
			    FW_CMD_EXEC | FW_EQ_CTRL_CMD_PFN(pf) |
			    FW_EQ_CTRL_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_EQ_CTRL_CMD_FREE | FW_LEN16(c));
	c.cmpliqid_eqid = htonl(FW_EQ_CTRL_CMD_EQID(eqid));
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_ofld_eq_free(struct adapter *adap, unsigned int mbox, unsigned int pf,
		    unsigned int vf, unsigned int eqid)
{
	struct fw_eq_ofld_cmd c;

	memset(&c, 0, sizeof(c));
	c.op_to_vfn = htonl(FW_CMD_OP(FW_EQ_OFLD_CMD) | FW_CMD_REQUEST |
			    FW_CMD_EXEC | FW_EQ_OFLD_CMD_PFN(pf) |
			    FW_EQ_OFLD_CMD_VFN(vf));
	c.alloc_to_len16 = htonl(FW_EQ_OFLD_CMD_FREE | FW_LEN16(c));
	c.eqid_pkd = htonl(FW_EQ_OFLD_CMD_EQID(eqid));
	return t4_wr_mbox(adap, mbox, &c, sizeof(c), NULL);
}

int t4_handle_fw_rpl(struct adapter *adap, const __be64 *rpl)
{
	u8 opcode = *(const u8 *)rpl;

	if (opcode == FW_PORT_CMD) {    /* link/module state change message */
		int speed = 0, fc = 0;
		const struct fw_port_cmd *p = (void *)rpl;
		int chan = FW_PORT_CMD_PORTID_GET(ntohl(p->op_to_portid));
		int port = adap->chan_map[chan];
		struct port_info *pi = adap2pinfo(adap, port);
		struct link_config *lc = &pi->link_cfg;
		u32 stat = ntohl(p->u.info.lstatus_to_modtype);
		int link_ok = (stat & FW_PORT_CMD_LSTATUS) != 0;
		u32 mod = FW_PORT_CMD_MODTYPE_GET(stat);

		if (stat & FW_PORT_CMD_RXPAUSE)
			fc |= PAUSE_RX;
		if (stat & FW_PORT_CMD_TXPAUSE)
			fc |= PAUSE_TX;
		if (stat & FW_PORT_CMD_LSPEED(FW_PORT_CAP_SPEED_100M))
			speed = SPEED_100;
		else if (stat & FW_PORT_CMD_LSPEED(FW_PORT_CAP_SPEED_1G))
			speed = SPEED_1000;
		else if (stat & FW_PORT_CMD_LSPEED(FW_PORT_CAP_SPEED_10G))
			speed = SPEED_10000;

		if (link_ok != lc->link_ok || speed != lc->speed ||
		    fc != lc->fc) {                    /* something changed */
			lc->link_ok = link_ok;
			lc->speed = speed;
			lc->fc = fc;
			t4_os_link_changed(adap, port, link_ok);
		}
		if (mod != pi->mod_type) {
			pi->mod_type = mod;
			t4_os_portmod_changed(adap, port);
		}
	}
	return 0;
}

static void __devinit get_pci_mode(struct adapter *adapter,
				   struct pci_params *p)
{
	u16 val;
	u32 pcie_cap = pci_pcie_cap(adapter->pdev);

	if (pcie_cap) {
		pci_read_config_word(adapter->pdev, pcie_cap + PCI_EXP_LNKSTA,
				     &val);
		p->speed = val & PCI_EXP_LNKSTA_CLS;
		p->width = (val & PCI_EXP_LNKSTA_NLW) >> 4;
	}
}

static void __devinit init_link_config(struct link_config *lc,
				       unsigned int caps)
{
	lc->supported = caps;
	lc->requested_speed = 0;
	lc->speed = 0;
	lc->requested_fc = lc->fc = PAUSE_RX | PAUSE_TX;
	if (lc->supported & FW_PORT_CAP_ANEG) {
		lc->advertising = lc->supported & ADVERT_MASK;
		lc->autoneg = AUTONEG_ENABLE;
		lc->requested_fc |= PAUSE_AUTONEG;
	} else {
		lc->advertising = 0;
		lc->autoneg = AUTONEG_DISABLE;
	}
}

static int __devinit wait_dev_ready(struct adapter *adap)
{
	if (t4_read_reg(adap, PL_WHOAMI) != 0xffffffff)
		return 0;
	msleep(500);
	return t4_read_reg(adap, PL_WHOAMI) != 0xffffffff ? 0 : -EIO;
}

int __devinit t4_prep_adapter(struct adapter *adapter)
{
	int ret;

	ret = wait_dev_ready(adapter);
	if (ret < 0)
		return ret;

	get_pci_mode(adapter, &adapter->params.pci);
	adapter->params.rev = t4_read_reg(adapter, PL_REV);

	ret = get_vpd_params(adapter, &adapter->params.vpd);
	if (ret < 0)
		return ret;

	init_cong_ctrl(adapter->params.a_wnd, adapter->params.b_wnd);

	/*
	 * Default port for debugging in case we can't reach FW.
	 */
	adapter->params.nports = 1;
	adapter->params.portvec = 1;
	return 0;
}

int __devinit t4_port_init(struct adapter *adap, int mbox, int pf, int vf)
{
	u8 addr[6];
	int ret, i, j = 0;
	struct fw_port_cmd c;

	memset(&c, 0, sizeof(c));

	for_each_port(adap, i) {
		unsigned int rss_size;
		struct port_info *p = adap2pinfo(adap, i);

		while ((adap->params.portvec & (1 << j)) == 0)
			j++;

		c.op_to_portid = htonl(FW_CMD_OP(FW_PORT_CMD) |
				       FW_CMD_REQUEST | FW_CMD_READ |
				       FW_PORT_CMD_PORTID(j));
		c.action_to_len16 = htonl(
			FW_PORT_CMD_ACTION(FW_PORT_ACTION_GET_PORT_INFO) |
			FW_LEN16(c));
		ret = t4_wr_mbox(adap, mbox, &c, sizeof(c), &c);
		if (ret)
			return ret;

		ret = t4_alloc_vi(adap, mbox, j, pf, vf, 1, addr, &rss_size);
		if (ret < 0)
			return ret;

		p->viid = ret;
		p->tx_chan = j;
		p->lport = j;
		p->rss_size = rss_size;
		memcpy(adap->port[i]->dev_addr, addr, ETH_ALEN);
		memcpy(adap->port[i]->perm_addr, addr, ETH_ALEN);

		ret = ntohl(c.u.info.lstatus_to_modtype);
		p->mdio_addr = (ret & FW_PORT_CMD_MDIOCAP) ?
			FW_PORT_CMD_MDIOADDR_GET(ret) : -1;
		p->port_type = FW_PORT_CMD_PTYPE_GET(ret);
		p->mod_type = FW_PORT_CMD_MODTYPE_GET(ret);

		init_link_config(&p->link_cfg, ntohs(c.u.info.pcap));
		j++;
	}
	return 0;
}

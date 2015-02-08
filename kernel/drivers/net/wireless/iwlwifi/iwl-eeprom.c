


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>

#include <net/mac80211.h>

#include "iwl-commands.h"
#include "iwl-dev.h"
#include "iwl-core.h"
#include "iwl-debug.h"
#include "iwl-eeprom.h"
#include "iwl-io.h"


/* 2.4 GHz */
const u8 iwl_eeprom_band_1[14] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14
};

/* 5.2 GHz bands */
static const u8 iwl_eeprom_band_2[] = {	/* 4915-5080MHz */
	183, 184, 185, 187, 188, 189, 192, 196, 7, 8, 11, 12, 16
};

static const u8 iwl_eeprom_band_3[] = {	/* 5170-5320MHz */
	34, 36, 38, 40, 42, 44, 46, 48, 52, 56, 60, 64
};

static const u8 iwl_eeprom_band_4[] = {	/* 5500-5700MHz */
	100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140
};

static const u8 iwl_eeprom_band_5[] = {	/* 5725-5825MHz */
	145, 149, 153, 157, 161, 165
};

static const u8 iwl_eeprom_band_6[] = {       /* 2.4 ht40 channel */
	1, 2, 3, 4, 5, 6, 7
};

static const u8 iwl_eeprom_band_7[] = {       /* 5.2 ht40 channel */
	36, 44, 52, 60, 100, 108, 116, 124, 132, 149, 157
};

struct iwl_txpwr_section {
	u32 offset;
	u8 count;
	enum ieee80211_band band;
	bool is_common;
	bool is_cck;
	bool is_ht40;
	u8 iwl_eeprom_section_channel[EEPROM_MAX_TXPOWER_SECTION_ELEMENTS];
};

static const struct iwl_txpwr_section enhinfo[] = {
	{ EEPROM_LB_CCK_20_COMMON, 1, IEEE80211_BAND_2GHZ, true, true, false },
	{ EEPROM_LB_OFDM_COMMON, 3, IEEE80211_BAND_2GHZ, true, false, false },
	{ EEPROM_HB_OFDM_COMMON, 3, IEEE80211_BAND_5GHZ, true, false, false },
	{ EEPROM_LB_OFDM_20_BAND, 8, IEEE80211_BAND_2GHZ,
		false, false, false,
		{1, 1, 2, 2, 10, 10, 11, 11 } },
	{ EEPROM_LB_OFDM_HT40_BAND, 5, IEEE80211_BAND_2GHZ,
		false, false, true,
		{ 1, 2, 6, 7, 9 } },
	{ EEPROM_HB_OFDM_20_BAND, 6, IEEE80211_BAND_5GHZ,
		false, false, false,
		{ 36, 64, 100, 36, 64, 100 } },
	{ EEPROM_HB_OFDM_HT40_BAND, 3, IEEE80211_BAND_5GHZ,
		false, false, true,
		{ 36, 60, 100 } },
	{ EEPROM_LB_OFDM_20_CHANNEL_13, 2, IEEE80211_BAND_2GHZ,
		false, false, false,
		{ 13, 13 } },
	{ EEPROM_HB_OFDM_20_CHANNEL_140, 2, IEEE80211_BAND_5GHZ,
		false, false, false,
		{ 140, 140 } },
	{ EEPROM_HB_OFDM_HT40_BAND_1, 2, IEEE80211_BAND_5GHZ,
		false, false, true,
		{ 132, 44 } },
};


int iwlcore_eeprom_verify_signature(struct iwl_priv *priv)
{
	u32 gp = iwl_read32(priv, CSR_EEPROM_GP) & CSR_EEPROM_GP_VALID_MSK;
	int ret = 0;

	IWL_DEBUG_INFO(priv, "EEPROM signature=0x%08x\n", gp);
	switch (gp) {
	case CSR_EEPROM_GP_BAD_SIG_EEP_GOOD_SIG_OTP:
		if (priv->nvm_device_type != NVM_DEVICE_TYPE_OTP) {
			IWL_ERR(priv, "EEPROM with bad signature: 0x%08x\n",
				gp);
			ret = -ENOENT;
		}
		break;
	case CSR_EEPROM_GP_GOOD_SIG_EEP_LESS_THAN_4K:
	case CSR_EEPROM_GP_GOOD_SIG_EEP_MORE_THAN_4K:
		if (priv->nvm_device_type != NVM_DEVICE_TYPE_EEPROM) {
			IWL_ERR(priv, "OTP with bad signature: 0x%08x\n", gp);
			ret = -ENOENT;
		}
		break;
	case CSR_EEPROM_GP_BAD_SIGNATURE_BOTH_EEP_AND_OTP:
	default:
		IWL_ERR(priv, "bad EEPROM/OTP signature, type=%s, "
			"EEPROM_GP=0x%08x\n",
			(priv->nvm_device_type == NVM_DEVICE_TYPE_OTP)
			? "OTP" : "EEPROM", gp);
		ret = -ENOENT;
		break;
	}
	return ret;
}
EXPORT_SYMBOL(iwlcore_eeprom_verify_signature);

static void iwl_set_otp_access(struct iwl_priv *priv, enum iwl_access_mode mode)
{
	u32 otpgp;

	otpgp = iwl_read32(priv, CSR_OTP_GP_REG);
	if (mode == IWL_OTP_ACCESS_ABSOLUTE)
		iwl_clear_bit(priv, CSR_OTP_GP_REG,
				CSR_OTP_GP_REG_OTP_ACCESS_MODE);
	else
		iwl_set_bit(priv, CSR_OTP_GP_REG,
				CSR_OTP_GP_REG_OTP_ACCESS_MODE);
}

static int iwlcore_get_nvm_type(struct iwl_priv *priv)
{
	u32 otpgp;
	int nvm_type;

	/* OTP only valid for CP/PP and after */
	switch (priv->hw_rev & CSR_HW_REV_TYPE_MSK) {
	case CSR_HW_REV_TYPE_NONE:
		IWL_ERR(priv, "Unknown hardware type\n");
		return -ENOENT;
	case CSR_HW_REV_TYPE_3945:
	case CSR_HW_REV_TYPE_4965:
	case CSR_HW_REV_TYPE_5300:
	case CSR_HW_REV_TYPE_5350:
	case CSR_HW_REV_TYPE_5100:
	case CSR_HW_REV_TYPE_5150:
		nvm_type = NVM_DEVICE_TYPE_EEPROM;
		break;
	default:
		otpgp = iwl_read32(priv, CSR_OTP_GP_REG);
		if (otpgp & CSR_OTP_GP_REG_DEVICE_SELECT)
			nvm_type = NVM_DEVICE_TYPE_OTP;
		else
			nvm_type = NVM_DEVICE_TYPE_EEPROM;
		break;
	}
	return  nvm_type;
}

int iwlcore_eeprom_acquire_semaphore(struct iwl_priv *priv)
{
	u16 count;
	int ret;

	for (count = 0; count < EEPROM_SEM_RETRY_LIMIT; count++) {
		/* Request semaphore */
		iwl_set_bit(priv, CSR_HW_IF_CONFIG_REG,
			    CSR_HW_IF_CONFIG_REG_BIT_EEPROM_OWN_SEM);

		/* See if we got it */
		ret = iwl_poll_bit(priv, CSR_HW_IF_CONFIG_REG,
				CSR_HW_IF_CONFIG_REG_BIT_EEPROM_OWN_SEM,
				CSR_HW_IF_CONFIG_REG_BIT_EEPROM_OWN_SEM,
				EEPROM_SEM_TIMEOUT);
		if (ret >= 0) {
			IWL_DEBUG_IO(priv, "Acquired semaphore after %d tries.\n",
				count+1);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL(iwlcore_eeprom_acquire_semaphore);

void iwlcore_eeprom_release_semaphore(struct iwl_priv *priv)
{
	iwl_clear_bit(priv, CSR_HW_IF_CONFIG_REG,
		CSR_HW_IF_CONFIG_REG_BIT_EEPROM_OWN_SEM);

}
EXPORT_SYMBOL(iwlcore_eeprom_release_semaphore);

const u8 *iwlcore_eeprom_query_addr(const struct iwl_priv *priv, size_t offset)
{
	BUG_ON(offset >= priv->cfg->eeprom_size);
	return &priv->eeprom[offset];
}
EXPORT_SYMBOL(iwlcore_eeprom_query_addr);

static int iwl_init_otp_access(struct iwl_priv *priv)
{
	int ret;

	/* Enable 40MHz radio clock */
	_iwl_write32(priv, CSR_GP_CNTRL,
		     _iwl_read32(priv, CSR_GP_CNTRL) |
		     CSR_GP_CNTRL_REG_FLAG_INIT_DONE);

	/* wait for clock to be ready */
	ret = iwl_poll_bit(priv, CSR_GP_CNTRL,
				  CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
				  CSR_GP_CNTRL_REG_FLAG_MAC_CLOCK_READY,
				  25000);
	if (ret < 0)
		IWL_ERR(priv, "Time out access OTP\n");
	else {
		iwl_set_bits_prph(priv, APMG_PS_CTRL_REG,
				  APMG_PS_CTRL_VAL_RESET_REQ);
		udelay(5);
		iwl_clear_bits_prph(priv, APMG_PS_CTRL_REG,
				    APMG_PS_CTRL_VAL_RESET_REQ);

		/*
		 * CSR auto clock gate disable bit -
		 * this is only applicable for HW with OTP shadow RAM
		 */
		if (priv->cfg->shadow_ram_support)
			iwl_set_bit(priv, CSR_DBG_LINK_PWR_MGMT_REG,
				CSR_RESET_LINK_PWR_MGMT_DISABLED);
	}
	return ret;
}

static int iwl_read_otp_word(struct iwl_priv *priv, u16 addr, __le16 *eeprom_data)
{
	int ret = 0;
	u32 r;
	u32 otpgp;

	_iwl_write32(priv, CSR_EEPROM_REG,
		     CSR_EEPROM_REG_MSK_ADDR & (addr << 1));
	ret = iwl_poll_bit(priv, CSR_EEPROM_REG,
				  CSR_EEPROM_REG_READ_VALID_MSK,
				  CSR_EEPROM_REG_READ_VALID_MSK,
				  IWL_EEPROM_ACCESS_TIMEOUT);
	if (ret < 0) {
		IWL_ERR(priv, "Time out reading OTP[%d]\n", addr);
		return ret;
	}
	r = _iwl_read_direct32(priv, CSR_EEPROM_REG);
	/* check for ECC errors: */
	otpgp = iwl_read32(priv, CSR_OTP_GP_REG);
	if (otpgp & CSR_OTP_GP_REG_ECC_UNCORR_STATUS_MSK) {
		/* stop in this case */
		/* set the uncorrectable OTP ECC bit for acknowledgement */
		iwl_set_bit(priv, CSR_OTP_GP_REG,
			CSR_OTP_GP_REG_ECC_UNCORR_STATUS_MSK);
		IWL_ERR(priv, "Uncorrectable OTP ECC error, abort OTP read\n");
		return -EINVAL;
	}
	if (otpgp & CSR_OTP_GP_REG_ECC_CORR_STATUS_MSK) {
		/* continue in this case */
		/* set the correctable OTP ECC bit for acknowledgement */
		iwl_set_bit(priv, CSR_OTP_GP_REG,
				CSR_OTP_GP_REG_ECC_CORR_STATUS_MSK);
		IWL_ERR(priv, "Correctable OTP ECC error, continue read\n");
	}
	*eeprom_data = cpu_to_le16(r >> 16);
	return 0;
}

static bool iwl_is_otp_empty(struct iwl_priv *priv)
{
	u16 next_link_addr = 0;
	__le16 link_value;
	bool is_empty = false;

	/* locate the beginning of OTP link list */
	if (!iwl_read_otp_word(priv, next_link_addr, &link_value)) {
		if (!link_value) {
			IWL_ERR(priv, "OTP is empty\n");
			is_empty = true;
		}
	} else {
		IWL_ERR(priv, "Unable to read first block of OTP list.\n");
		is_empty = true;
	}

	return is_empty;
}


static int iwl_find_otp_image(struct iwl_priv *priv,
					u16 *validblockaddr)
{
	u16 next_link_addr = 0, valid_addr;
	__le16 link_value = 0;
	int usedblocks = 0;

	/* set addressing mode to absolute to traverse the link list */
	iwl_set_otp_access(priv, IWL_OTP_ACCESS_ABSOLUTE);

	/* checking for empty OTP or error */
	if (iwl_is_otp_empty(priv))
		return -EINVAL;

	/*
	 * start traverse link list
	 * until reach the max number of OTP blocks
	 * different devices have different number of OTP blocks
	 */
	do {
		/* save current valid block address
		 * check for more block on the link list
		 */
		valid_addr = next_link_addr;
		next_link_addr = le16_to_cpu(link_value) * sizeof(u16);
		IWL_DEBUG_INFO(priv, "OTP blocks %d addr 0x%x\n",
			       usedblocks, next_link_addr);
		if (iwl_read_otp_word(priv, next_link_addr, &link_value))
			return -EINVAL;
		if (!link_value) {
			/*
			 * reach the end of link list, return success and
			 * set address point to the starting address
			 * of the image
			 */
			*validblockaddr = valid_addr;
			/* skip first 2 bytes (link list pointer) */
			*validblockaddr += 2;
			return 0;
		}
		/* more in the link list, continue */
		usedblocks++;
	} while (usedblocks <= priv->cfg->max_ll_items);

	/* OTP has no valid blocks */
	IWL_DEBUG_INFO(priv, "OTP has no valid blocks\n");
	return -EINVAL;
}

int iwl_eeprom_init(struct iwl_priv *priv)
{
	__le16 *e;
	u32 gp = iwl_read32(priv, CSR_EEPROM_GP);
	int sz;
	int ret;
	u16 addr;
	u16 validblockaddr = 0;
	u16 cache_addr = 0;

	priv->nvm_device_type = iwlcore_get_nvm_type(priv);
	if (priv->nvm_device_type == -ENOENT)
		return -ENOENT;
	/* allocate eeprom */
	IWL_DEBUG_INFO(priv, "NVM size = %d\n", priv->cfg->eeprom_size);
	sz = priv->cfg->eeprom_size;
	priv->eeprom = kzalloc(sz, GFP_KERNEL);
	if (!priv->eeprom) {
		ret = -ENOMEM;
		goto alloc_err;
	}
	e = (__le16 *)priv->eeprom;

	priv->cfg->ops->lib->apm_ops.init(priv);

	ret = priv->cfg->ops->lib->eeprom_ops.verify_signature(priv);
	if (ret < 0) {
		IWL_ERR(priv, "EEPROM not found, EEPROM_GP=0x%08x\n", gp);
		ret = -ENOENT;
		goto err;
	}

	/* Make sure driver (instead of uCode) is allowed to read EEPROM */
	ret = priv->cfg->ops->lib->eeprom_ops.acquire_semaphore(priv);
	if (ret < 0) {
		IWL_ERR(priv, "Failed to acquire EEPROM semaphore.\n");
		ret = -ENOENT;
		goto err;
	}

	if (priv->nvm_device_type == NVM_DEVICE_TYPE_OTP) {

		ret = iwl_init_otp_access(priv);
		if (ret) {
			IWL_ERR(priv, "Failed to initialize OTP access.\n");
			ret = -ENOENT;
			goto done;
		}
		_iwl_write32(priv, CSR_EEPROM_GP,
			     iwl_read32(priv, CSR_EEPROM_GP) &
			     ~CSR_EEPROM_GP_IF_OWNER_MSK);

		iwl_set_bit(priv, CSR_OTP_GP_REG,
			     CSR_OTP_GP_REG_ECC_CORR_STATUS_MSK |
			     CSR_OTP_GP_REG_ECC_UNCORR_STATUS_MSK);
		/* traversing the linked list if no shadow ram supported */
		if (!priv->cfg->shadow_ram_support) {
			if (iwl_find_otp_image(priv, &validblockaddr)) {
				ret = -ENOENT;
				goto done;
			}
		}
		for (addr = validblockaddr; addr < validblockaddr + sz;
		     addr += sizeof(u16)) {
			__le16 eeprom_data;

			ret = iwl_read_otp_word(priv, addr, &eeprom_data);
			if (ret)
				goto done;
			e[cache_addr / 2] = eeprom_data;
			cache_addr += sizeof(u16);
		}
	} else {
		/* eeprom is an array of 16bit values */
		for (addr = 0; addr < sz; addr += sizeof(u16)) {
			u32 r;

			_iwl_write32(priv, CSR_EEPROM_REG,
				     CSR_EEPROM_REG_MSK_ADDR & (addr << 1));

			ret = iwl_poll_bit(priv, CSR_EEPROM_REG,
						  CSR_EEPROM_REG_READ_VALID_MSK,
						  CSR_EEPROM_REG_READ_VALID_MSK,
						  IWL_EEPROM_ACCESS_TIMEOUT);
			if (ret < 0) {
				IWL_ERR(priv, "Time out reading EEPROM[%d]\n", addr);
				goto done;
			}
			r = _iwl_read_direct32(priv, CSR_EEPROM_REG);
			e[addr / 2] = cpu_to_le16(r >> 16);
		}
	}

	IWL_DEBUG_INFO(priv, "NVM Type: %s, version: 0x%x\n",
		       (priv->nvm_device_type == NVM_DEVICE_TYPE_OTP)
		       ? "OTP" : "EEPROM",
		       iwl_eeprom_query16(priv, EEPROM_VERSION));

	ret = 0;
done:
	priv->cfg->ops->lib->eeprom_ops.release_semaphore(priv);

err:
	if (ret)
		iwl_eeprom_free(priv);
	/* Reset chip to save power until we load uCode during "up". */
	priv->cfg->ops->lib->apm_ops.stop(priv);
alloc_err:
	return ret;
}
EXPORT_SYMBOL(iwl_eeprom_init);

void iwl_eeprom_free(struct iwl_priv *priv)
{
	kfree(priv->eeprom);
	priv->eeprom = NULL;
}
EXPORT_SYMBOL(iwl_eeprom_free);

int iwl_eeprom_check_version(struct iwl_priv *priv)
{
	u16 eeprom_ver;
	u16 calib_ver;

	eeprom_ver = iwl_eeprom_query16(priv, EEPROM_VERSION);
	calib_ver = priv->cfg->ops->lib->eeprom_ops.calib_version(priv);

	if (eeprom_ver < priv->cfg->eeprom_ver ||
	    calib_ver < priv->cfg->eeprom_calib_ver)
		goto err;

	return 0;
err:
	IWL_ERR(priv, "Unsupported (too old) EEPROM VER=0x%x < 0x%x CALIB=0x%x < 0x%x\n",
		  eeprom_ver, priv->cfg->eeprom_ver,
		  calib_ver,  priv->cfg->eeprom_calib_ver);
	return -EINVAL;

}
EXPORT_SYMBOL(iwl_eeprom_check_version);

const u8 *iwl_eeprom_query_addr(const struct iwl_priv *priv, size_t offset)
{
	return priv->cfg->ops->lib->eeprom_ops.query_addr(priv, offset);
}
EXPORT_SYMBOL(iwl_eeprom_query_addr);

u16 iwl_eeprom_query16(const struct iwl_priv *priv, size_t offset)
{
	if (!priv->eeprom)
		return 0;
	return (u16)priv->eeprom[offset] | ((u16)priv->eeprom[offset + 1] << 8);
}
EXPORT_SYMBOL(iwl_eeprom_query16);

void iwl_eeprom_get_mac(const struct iwl_priv *priv, u8 *mac)
{
	const u8 *addr = priv->cfg->ops->lib->eeprom_ops.query_addr(priv,
					EEPROM_MAC_ADDRESS);
	memcpy(mac, addr, ETH_ALEN);
}
EXPORT_SYMBOL(iwl_eeprom_get_mac);

static void iwl_init_band_reference(const struct iwl_priv *priv,
			int eep_band, int *eeprom_ch_count,
			const struct iwl_eeprom_channel **eeprom_ch_info,
			const u8 **eeprom_ch_index)
{
	u32 offset = priv->cfg->ops->lib->
			eeprom_ops.regulatory_bands[eep_band - 1];
	switch (eep_band) {
	case 1:		/* 2.4GHz band */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_1);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_1;
		break;
	case 2:		/* 4.9GHz band */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_2);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_2;
		break;
	case 3:		/* 5.2GHz band */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_3);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_3;
		break;
	case 4:		/* 5.5GHz band */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_4);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_4;
		break;
	case 5:		/* 5.7GHz band */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_5);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_5;
		break;
	case 6:		/* 2.4GHz ht40 channels */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_6);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_6;
		break;
	case 7:		/* 5 GHz ht40 channels */
		*eeprom_ch_count = ARRAY_SIZE(iwl_eeprom_band_7);
		*eeprom_ch_info = (struct iwl_eeprom_channel *)
				iwl_eeprom_query_addr(priv, offset);
		*eeprom_ch_index = iwl_eeprom_band_7;
		break;
	default:
		BUG();
		return;
	}
}

#define CHECK_AND_PRINT(x) ((eeprom_ch->flags & EEPROM_CHANNEL_##x) \
			    ? # x " " : "")

static int iwl_mod_ht40_chan_info(struct iwl_priv *priv,
			      enum ieee80211_band band, u16 channel,
			      const struct iwl_eeprom_channel *eeprom_ch,
			      u8 clear_ht40_extension_channel)
{
	struct iwl_channel_info *ch_info;

	ch_info = (struct iwl_channel_info *)
			iwl_get_channel_info(priv, band, channel);

	if (!is_channel_valid(ch_info))
		return -1;

	IWL_DEBUG_INFO(priv, "HT40 Ch. %d [%sGHz] %s%s%s%s%s(0x%02x %ddBm):"
			" Ad-Hoc %ssupported\n",
			ch_info->channel,
			is_channel_a_band(ch_info) ?
			"5.2" : "2.4",
			CHECK_AND_PRINT(IBSS),
			CHECK_AND_PRINT(ACTIVE),
			CHECK_AND_PRINT(RADAR),
			CHECK_AND_PRINT(WIDE),
			CHECK_AND_PRINT(DFS),
			eeprom_ch->flags,
			eeprom_ch->max_power_avg,
			((eeprom_ch->flags & EEPROM_CHANNEL_IBSS)
			 && !(eeprom_ch->flags & EEPROM_CHANNEL_RADAR)) ?
			"" : "not ");

	ch_info->ht40_eeprom = *eeprom_ch;
	ch_info->ht40_max_power_avg = eeprom_ch->max_power_avg;
	ch_info->ht40_flags = eeprom_ch->flags;
	if (eeprom_ch->flags & EEPROM_CHANNEL_VALID)
		ch_info->ht40_extension_channel &= ~clear_ht40_extension_channel;

	return 0;
}

static s8 iwl_get_max_txpower_avg(struct iwl_priv *priv,
		struct iwl_eeprom_enhanced_txpwr *enhanced_txpower,
		int element, s8 *max_txpower_in_half_dbm)
{
	s8 max_txpower_avg = 0; /* (dBm) */

	IWL_DEBUG_INFO(priv, "%d - "
			"chain_a: %d dB chain_b: %d dB "
			"chain_c: %d dB mimo2: %d dB mimo3: %d dB\n",
			element,
			enhanced_txpower[element].chain_a_max >> 1,
			enhanced_txpower[element].chain_b_max >> 1,
			enhanced_txpower[element].chain_c_max >> 1,
			enhanced_txpower[element].mimo2_max >> 1,
			enhanced_txpower[element].mimo3_max >> 1);
	/* Take the highest tx power from any valid chains */
	if ((priv->cfg->valid_tx_ant & ANT_A) &&
	    (enhanced_txpower[element].chain_a_max > max_txpower_avg))
		max_txpower_avg = enhanced_txpower[element].chain_a_max;
	if ((priv->cfg->valid_tx_ant & ANT_B) &&
	    (enhanced_txpower[element].chain_b_max > max_txpower_avg))
		max_txpower_avg = enhanced_txpower[element].chain_b_max;
	if ((priv->cfg->valid_tx_ant & ANT_C) &&
	    (enhanced_txpower[element].chain_c_max > max_txpower_avg))
		max_txpower_avg = enhanced_txpower[element].chain_c_max;
	if (((priv->cfg->valid_tx_ant == ANT_AB) |
	    (priv->cfg->valid_tx_ant == ANT_BC) |
	    (priv->cfg->valid_tx_ant == ANT_AC)) &&
	    (enhanced_txpower[element].mimo2_max > max_txpower_avg))
		max_txpower_avg =  enhanced_txpower[element].mimo2_max;
	if ((priv->cfg->valid_tx_ant == ANT_ABC) &&
	    (enhanced_txpower[element].mimo3_max > max_txpower_avg))
		max_txpower_avg = enhanced_txpower[element].mimo3_max;

	/*
	 * max. tx power in EEPROM is in 1/2 dBm format
	 * convert from 1/2 dBm to dBm (round-up convert)
	 * but we also do not want to loss 1/2 dBm resolution which
	 * will impact performance
	 */
	*max_txpower_in_half_dbm = max_txpower_avg;
	return (max_txpower_avg & 0x01) + (max_txpower_avg >> 1);
}

static s8 iwl_update_common_txpower(struct iwl_priv *priv,
		struct iwl_eeprom_enhanced_txpwr *enhanced_txpower,
		int section, int element, s8 *max_txpower_in_half_dbm)
{
	struct iwl_channel_info *ch_info;
	int ch;
	bool is_ht40 = false;
	s8 max_txpower_avg; /* (dBm) */

	/* it is common section, contain all type (Legacy, HT and HT40)
	 * based on the element in the section to determine
	 * is it HT 40 or not
	 */
	if (element == EEPROM_TXPOWER_COMMON_HT40_INDEX)
		is_ht40 = true;
	max_txpower_avg =
		iwl_get_max_txpower_avg(priv, enhanced_txpower,
					element, max_txpower_in_half_dbm);

	ch_info = priv->channel_info;

	for (ch = 0; ch < priv->channel_count; ch++) {
		/* find matching band and update tx power if needed */
		if ((ch_info->band == enhinfo[section].band) &&
		    (ch_info->max_power_avg < max_txpower_avg) &&
		    (!is_ht40)) {
			/* Update regulatory-based run-time data */
			ch_info->max_power_avg = ch_info->curr_txpow =
				max_txpower_avg;
			ch_info->scan_power = max_txpower_avg;
		}
		if ((ch_info->band == enhinfo[section].band) && is_ht40 &&
		    (ch_info->ht40_max_power_avg < max_txpower_avg)) {
			/* Update regulatory-based run-time data */
			ch_info->ht40_max_power_avg = max_txpower_avg;
		}
		ch_info++;
	}
	return max_txpower_avg;
}

static s8 iwl_update_channel_txpower(struct iwl_priv *priv,
		struct iwl_eeprom_enhanced_txpwr *enhanced_txpower,
		int section, int element, s8 *max_txpower_in_half_dbm)
{
	struct iwl_channel_info *ch_info;
	int ch;
	u8 channel;
	s8 max_txpower_avg; /* (dBm) */

	channel = enhinfo[section].iwl_eeprom_section_channel[element];
	max_txpower_avg =
		iwl_get_max_txpower_avg(priv, enhanced_txpower,
					element, max_txpower_in_half_dbm);

	ch_info = priv->channel_info;
	for (ch = 0; ch < priv->channel_count; ch++) {
		/* find matching channel and update tx power if needed */
		if (ch_info->channel == channel) {
			if ((ch_info->max_power_avg < max_txpower_avg) &&
			    (!enhinfo[section].is_ht40)) {
				/* Update regulatory-based run-time data */
				ch_info->max_power_avg = max_txpower_avg;
				ch_info->curr_txpow = max_txpower_avg;
				ch_info->scan_power = max_txpower_avg;
			}
			if ((enhinfo[section].is_ht40) &&
			    (ch_info->ht40_max_power_avg < max_txpower_avg)) {
				/* Update regulatory-based run-time data */
				ch_info->ht40_max_power_avg = max_txpower_avg;
			}
			break;
		}
		ch_info++;
	}
	return max_txpower_avg;
}

void iwlcore_eeprom_enhanced_txpower(struct iwl_priv *priv)
{
	int eeprom_section_count = 0;
	int section, element;
	struct iwl_eeprom_enhanced_txpwr *enhanced_txpower;
	u32 offset;
	s8 max_txpower_avg; /* (dBm) */
	s8 max_txpower_in_half_dbm; /* (half-dBm) */

	/* Loop through all the sections
	 * adjust bands and channel's max tx power
	 * Set the tx_power_user_lmt to the highest power
	 * supported by any channels and chains
	 */
	for (section = 0; section < ARRAY_SIZE(enhinfo); section++) {
		eeprom_section_count = enhinfo[section].count;
		offset = enhinfo[section].offset;
		enhanced_txpower = (struct iwl_eeprom_enhanced_txpwr *)
				iwl_eeprom_query_addr(priv, offset);

		/*
		 * check for valid entry -
		 * different version of EEPROM might contain different set
		 * of enhanced tx power table
		 * always check for valid entry before process
		 * the information
		 */
		if (!enhanced_txpower->common || enhanced_txpower->reserved)
			continue;

		for (element = 0; element < eeprom_section_count; element++) {
			if (enhinfo[section].is_common)
				max_txpower_avg =
					iwl_update_common_txpower(priv,
						enhanced_txpower, section,
						element,
						&max_txpower_in_half_dbm);
			else
				max_txpower_avg =
					iwl_update_channel_txpower(priv,
						enhanced_txpower, section,
						element,
						&max_txpower_in_half_dbm);

			/* Update the tx_power_user_lmt to the highest power
			 * supported by any channel */
			if (max_txpower_avg > priv->tx_power_user_lmt)
				priv->tx_power_user_lmt = max_txpower_avg;

			/*
			 * Update the tx_power_lmt_in_half_dbm to
			 * the highest power supported by any channel
			 */
			if (max_txpower_in_half_dbm >
			    priv->tx_power_lmt_in_half_dbm)
				priv->tx_power_lmt_in_half_dbm =
					max_txpower_in_half_dbm;
		}
	}
}
EXPORT_SYMBOL(iwlcore_eeprom_enhanced_txpower);

#define CHECK_AND_PRINT_I(x) ((eeprom_ch_info[ch].flags & EEPROM_CHANNEL_##x) \
			    ? # x " " : "")

int iwl_init_channel_map(struct iwl_priv *priv)
{
	int eeprom_ch_count = 0;
	const u8 *eeprom_ch_index = NULL;
	const struct iwl_eeprom_channel *eeprom_ch_info = NULL;
	int band, ch;
	struct iwl_channel_info *ch_info;

	if (priv->channel_count) {
		IWL_DEBUG_INFO(priv, "Channel map already initialized.\n");
		return 0;
	}

	IWL_DEBUG_INFO(priv, "Initializing regulatory info from EEPROM\n");

	priv->channel_count =
	    ARRAY_SIZE(iwl_eeprom_band_1) +
	    ARRAY_SIZE(iwl_eeprom_band_2) +
	    ARRAY_SIZE(iwl_eeprom_band_3) +
	    ARRAY_SIZE(iwl_eeprom_band_4) +
	    ARRAY_SIZE(iwl_eeprom_band_5);

	IWL_DEBUG_INFO(priv, "Parsing data for %d channels.\n", priv->channel_count);

	priv->channel_info = kzalloc(sizeof(struct iwl_channel_info) *
				     priv->channel_count, GFP_KERNEL);
	if (!priv->channel_info) {
		IWL_ERR(priv, "Could not allocate channel_info\n");
		priv->channel_count = 0;
		return -ENOMEM;
	}

	ch_info = priv->channel_info;

	/* Loop through the 5 EEPROM bands adding them in order to the
	 * channel map we maintain (that contains additional information than
	 * what just in the EEPROM) */
	for (band = 1; band <= 5; band++) {

		iwl_init_band_reference(priv, band, &eeprom_ch_count,
					&eeprom_ch_info, &eeprom_ch_index);

		/* Loop through each band adding each of the channels */
		for (ch = 0; ch < eeprom_ch_count; ch++) {
			ch_info->channel = eeprom_ch_index[ch];
			ch_info->band = (band == 1) ? IEEE80211_BAND_2GHZ :
			    IEEE80211_BAND_5GHZ;

			/* permanently store EEPROM's channel regulatory flags
			 *   and max power in channel info database. */
			ch_info->eeprom = eeprom_ch_info[ch];

			/* Copy the run-time flags so they are there even on
			 * invalid channels */
			ch_info->flags = eeprom_ch_info[ch].flags;
			/* First write that ht40 is not enabled, and then enable
			 * one by one */
			ch_info->ht40_extension_channel =
					IEEE80211_CHAN_NO_HT40;

			if (!(is_channel_valid(ch_info))) {
				IWL_DEBUG_INFO(priv, "Ch. %d Flags %x [%sGHz] - "
					       "No traffic\n",
					       ch_info->channel,
					       ch_info->flags,
					       is_channel_a_band(ch_info) ?
					       "5.2" : "2.4");
				ch_info++;
				continue;
			}

			/* Initialize regulatory-based run-time data */
			ch_info->max_power_avg = ch_info->curr_txpow =
			    eeprom_ch_info[ch].max_power_avg;
			ch_info->scan_power = eeprom_ch_info[ch].max_power_avg;
			ch_info->min_power = 0;

			IWL_DEBUG_INFO(priv, "Ch. %d [%sGHz] %s%s%s%s%s%s(0x%02x %ddBm):"
				       " Ad-Hoc %ssupported\n",
				       ch_info->channel,
				       is_channel_a_band(ch_info) ?
				       "5.2" : "2.4",
				       CHECK_AND_PRINT_I(VALID),
				       CHECK_AND_PRINT_I(IBSS),
				       CHECK_AND_PRINT_I(ACTIVE),
				       CHECK_AND_PRINT_I(RADAR),
				       CHECK_AND_PRINT_I(WIDE),
				       CHECK_AND_PRINT_I(DFS),
				       eeprom_ch_info[ch].flags,
				       eeprom_ch_info[ch].max_power_avg,
				       ((eeprom_ch_info[ch].
					 flags & EEPROM_CHANNEL_IBSS)
					&& !(eeprom_ch_info[ch].
					     flags & EEPROM_CHANNEL_RADAR))
				       ? "" : "not ");

			/* Set the tx_power_user_lmt to the highest power
			 * supported by any channel */
			if (eeprom_ch_info[ch].max_power_avg >
						priv->tx_power_user_lmt)
				priv->tx_power_user_lmt =
				    eeprom_ch_info[ch].max_power_avg;

			ch_info++;
		}
	}

	/* Check if we do have HT40 channels */
	if (priv->cfg->ops->lib->eeprom_ops.regulatory_bands[5] ==
	    EEPROM_REGULATORY_BAND_NO_HT40 &&
	    priv->cfg->ops->lib->eeprom_ops.regulatory_bands[6] ==
	    EEPROM_REGULATORY_BAND_NO_HT40)
		return 0;

	/* Two additional EEPROM bands for 2.4 and 5 GHz HT40 channels */
	for (band = 6; band <= 7; band++) {
		enum ieee80211_band ieeeband;

		iwl_init_band_reference(priv, band, &eeprom_ch_count,
					&eeprom_ch_info, &eeprom_ch_index);

		/* EEPROM band 6 is 2.4, band 7 is 5 GHz */
		ieeeband =
			(band == 6) ? IEEE80211_BAND_2GHZ : IEEE80211_BAND_5GHZ;

		/* Loop through each band adding each of the channels */
		for (ch = 0; ch < eeprom_ch_count; ch++) {
			/* Set up driver's info for lower half */
			iwl_mod_ht40_chan_info(priv, ieeeband,
						eeprom_ch_index[ch],
						&eeprom_ch_info[ch],
						IEEE80211_CHAN_NO_HT40PLUS);

			/* Set up driver's info for upper half */
			iwl_mod_ht40_chan_info(priv, ieeeband,
						eeprom_ch_index[ch] + 4,
						&eeprom_ch_info[ch],
						IEEE80211_CHAN_NO_HT40MINUS);
		}
	}

	/* for newer device (6000 series and up)
	 * EEPROM contain enhanced tx power information
	 * driver need to process addition information
	 * to determine the max channel tx power limits
	 */
	if (priv->cfg->ops->lib->eeprom_ops.update_enhanced_txpower)
		priv->cfg->ops->lib->eeprom_ops.update_enhanced_txpower(priv);

	return 0;
}
EXPORT_SYMBOL(iwl_init_channel_map);

void iwl_free_channel_map(struct iwl_priv *priv)
{
	kfree(priv->channel_info);
	priv->channel_count = 0;
}
EXPORT_SYMBOL(iwl_free_channel_map);

const struct iwl_channel_info *iwl_get_channel_info(const struct iwl_priv *priv,
					enum ieee80211_band band, u16 channel)
{
	int i;

	switch (band) {
	case IEEE80211_BAND_5GHZ:
		for (i = 14; i < priv->channel_count; i++) {
			if (priv->channel_info[i].channel == channel)
				return &priv->channel_info[i];
		}
		break;
	case IEEE80211_BAND_2GHZ:
		if (channel >= 1 && channel <= 14)
			return &priv->channel_info[channel - 1];
		break;
	default:
		BUG();
	}

	return NULL;
}
EXPORT_SYMBOL(iwl_get_channel_info);


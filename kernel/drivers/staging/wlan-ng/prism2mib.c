

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wireless.h>
#include <linux/netdevice.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <asm/byteorder.h>
#include <linux/usb.h>
#include <linux/bitops.h>

#include "p80211types.h"
#include "p80211hdr.h"
#include "p80211mgmt.h"
#include "p80211conv.h"
#include "p80211msg.h"
#include "p80211netdev.h"
#include "p80211metadef.h"
#include "p80211metastruct.h"
#include "hfa384x.h"
#include "prism2mgmt.h"

#define MIB_TMP_MAXLEN    200	/* Max length of RID record (in bytes). */

#define  F_STA        0x1	/* MIB is supported on stations. */
#define  F_READ       0x2	/* MIB may be read. */
#define  F_WRITE      0x4	/* MIB may be written. */

typedef struct mibrec {
	u32 did;
	u16 flag;
	u16 parm1;
	u16 parm2;
	u16 parm3;
	int (*func) (struct mibrec *mib,
		     int isget,
		     wlandevice_t *wlandev,
		     hfa384x_t *hw,
		     p80211msg_dot11req_mibset_t *msg, void *data);
} mibrec_t;

static int prism2mib_bytearea2pstr(mibrec_t *mib,
				   int isget,
				   wlandevice_t *wlandev,
				   hfa384x_t *hw,
				   p80211msg_dot11req_mibset_t *msg,
				   void *data);

static int prism2mib_uint32(mibrec_t *mib,
			    int isget,
			    wlandevice_t *wlandev,
			    hfa384x_t *hw,
			    p80211msg_dot11req_mibset_t *msg, void *data);

static int prism2mib_flag(mibrec_t *mib,
			  int isget,
			  wlandevice_t *wlandev,
			  hfa384x_t *hw,
			  p80211msg_dot11req_mibset_t *msg, void *data);

static int prism2mib_wepdefaultkey(mibrec_t *mib,
				   int isget,
				   wlandevice_t *wlandev,
				   hfa384x_t *hw,
				   p80211msg_dot11req_mibset_t *msg,
				   void *data);

static int prism2mib_privacyinvoked(mibrec_t *mib,
				    int isget,
				    wlandevice_t *wlandev,
				    hfa384x_t *hw,
				    p80211msg_dot11req_mibset_t *msg,
				    void *data);

static int prism2mib_excludeunencrypted(mibrec_t *mib,
					int isget,
					wlandevice_t *wlandev,
					hfa384x_t *hw,
					p80211msg_dot11req_mibset_t *msg,
					void *data);

static int prism2mib_fragmentationthreshold(mibrec_t *mib,
					    int isget,
					    wlandevice_t *wlandev,
					    hfa384x_t *hw,
					    p80211msg_dot11req_mibset_t *msg,
					    void *data);

static int prism2mib_priv(mibrec_t *mib,
			  int isget,
			  wlandevice_t *wlandev,
			  hfa384x_t *hw,
			  p80211msg_dot11req_mibset_t *msg, void *data);

static mibrec_t mibtab[] = {

	/* dot11smt MIB's */
	{DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey0,
	 F_STA | F_WRITE,
	 HFA384x_RID_CNFWEPDEFAULTKEY0, 0, 0,
	 prism2mib_wepdefaultkey},
	{DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey1,
	 F_STA | F_WRITE,
	 HFA384x_RID_CNFWEPDEFAULTKEY1, 0, 0,
	 prism2mib_wepdefaultkey},
	{DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey2,
	 F_STA | F_WRITE,
	 HFA384x_RID_CNFWEPDEFAULTKEY2, 0, 0,
	 prism2mib_wepdefaultkey},
	{DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey3,
	 F_STA | F_WRITE,
	 HFA384x_RID_CNFWEPDEFAULTKEY3, 0, 0,
	 prism2mib_wepdefaultkey},
	{DIDmib_dot11smt_dot11PrivacyTable_dot11PrivacyInvoked,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_CNFWEPFLAGS, HFA384x_WEPFLAGS_PRIVINVOKED, 0,
	 prism2mib_privacyinvoked},
	{DIDmib_dot11smt_dot11PrivacyTable_dot11WEPDefaultKeyID,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_CNFWEPDEFAULTKEYID, 0, 0,
	 prism2mib_uint32},
	{DIDmib_dot11smt_dot11PrivacyTable_dot11ExcludeUnencrypted,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_CNFWEPFLAGS, HFA384x_WEPFLAGS_EXCLUDE, 0,
	 prism2mib_excludeunencrypted},

	/* dot11mac MIB's */

	{DIDmib_dot11mac_dot11OperationTable_dot11MACAddress,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_CNFOWNMACADDR, HFA384x_RID_CNFOWNMACADDR_LEN, 0,
	 prism2mib_bytearea2pstr},
	{DIDmib_dot11mac_dot11OperationTable_dot11RTSThreshold,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_RTSTHRESH, 0, 0,
	 prism2mib_uint32},
	{DIDmib_dot11mac_dot11OperationTable_dot11ShortRetryLimit,
	 F_STA | F_READ,
	 HFA384x_RID_SHORTRETRYLIMIT, 0, 0,
	 prism2mib_uint32},
	{DIDmib_dot11mac_dot11OperationTable_dot11LongRetryLimit,
	 F_STA | F_READ,
	 HFA384x_RID_LONGRETRYLIMIT, 0, 0,
	 prism2mib_uint32},
	{DIDmib_dot11mac_dot11OperationTable_dot11FragmentationThreshold,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_FRAGTHRESH, 0, 0,
	 prism2mib_fragmentationthreshold},
	{DIDmib_dot11mac_dot11OperationTable_dot11MaxTransmitMSDULifetime,
	 F_STA | F_READ,
	 HFA384x_RID_MAXTXLIFETIME, 0, 0,
	 prism2mib_uint32},

	/* dot11phy MIB's */

	{DIDmib_dot11phy_dot11PhyDSSSTable_dot11CurrentChannel,
	 F_STA | F_READ,
	 HFA384x_RID_CURRENTCHANNEL, 0, 0,
	 prism2mib_uint32},
	{DIDmib_dot11phy_dot11PhyTxPowerTable_dot11CurrentTxPowerLevel,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_TXPOWERMAX, 0, 0,
	 prism2mib_uint32},

	/* p2Static MIB's */

	{DIDmib_p2_p2Static_p2CnfPortType,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_CNFPORTTYPE, 0, 0,
	 prism2mib_uint32},

	/* p2MAC MIB's */

	{DIDmib_p2_p2MAC_p2CurrentTxRate,
	 F_STA | F_READ,
	 HFA384x_RID_CURRENTTXRATE, 0, 0,
	 prism2mib_uint32},

	/* And finally, lnx mibs */
	{DIDmib_lnx_lnxConfigTable_lnxRSNAIE,
	 F_STA | F_READ | F_WRITE,
	 HFA384x_RID_CNFWPADATA, 0, 0,
	 prism2mib_priv},
	{0, 0, 0, 0, 0, NULL}
};


int prism2mgmt_mibset_mibget(wlandevice_t *wlandev, void *msgp)
{
	hfa384x_t *hw = wlandev->priv;
	int result, isget;
	mibrec_t *mib;

	u16 which;

	p80211msg_dot11req_mibset_t *msg = msgp;
	p80211itemd_t *mibitem;

	msg->resultcode.status = P80211ENUM_msgitem_status_data_ok;
	msg->resultcode.data = P80211ENUM_resultcode_success;

	/*
	 ** Determine if this is an Access Point or a station.
	 */

	which = F_STA;

	/*
	 ** Find the MIB in the MIB table.  Note that a MIB may be in the
	 ** table twice...once for an AP and once for a station.  Make sure
	 ** to get the correct one.  Note that DID=0 marks the end of the
	 ** MIB table.
	 */

	mibitem = (p80211itemd_t *) msg->mibattribute.data;

	for (mib = mibtab; mib->did != 0; mib++)
		if (mib->did == mibitem->did && (mib->flag & which))
			break;

	if (mib->did == 0) {
		msg->resultcode.data = P80211ENUM_resultcode_not_supported;
		goto done;
	}

	/*
	 ** Determine if this is a "mibget" or a "mibset".  If this is a
	 ** "mibget", then make sure that the MIB may be read.  Otherwise,
	 ** this is a "mibset" so make make sure that the MIB may be written.
	 */

	isget = (msg->msgcode == DIDmsg_dot11req_mibget);

	if (isget) {
		if (!(mib->flag & F_READ)) {
			msg->resultcode.data =
			    P80211ENUM_resultcode_cant_get_writeonly_mib;
			goto done;
		}
	} else {
		if (!(mib->flag & F_WRITE)) {
			msg->resultcode.data =
			    P80211ENUM_resultcode_cant_set_readonly_mib;
			goto done;
		}
	}

	/*
	 ** Execute the MIB function.  If things worked okay, then make
	 ** sure that the MIB function also worked okay.  If so, and this
	 ** is a "mibget", then the status value must be set for both the
	 ** "mibattribute" parameter and the mib item within the data
	 ** portion of the "mibattribute".
	 */

	result = mib->func(mib, isget, wlandev, hw, msg, (void *)mibitem->data);

	if (msg->resultcode.data == P80211ENUM_resultcode_success) {
		if (result != 0) {
			pr_debug("get/set failure, result=%d\n", result);
			msg->resultcode.data =
			    P80211ENUM_resultcode_implementation_failure;
		} else {
			if (isget) {
				msg->mibattribute.status =
				    P80211ENUM_msgitem_status_data_ok;
				mibitem->status =
				    P80211ENUM_msgitem_status_data_ok;
			}
		}
	}

done:
	return 0;
}


static int prism2mib_bytearea2pstr(mibrec_t *mib,
				   int isget,
				   wlandevice_t *wlandev,
				   hfa384x_t *hw,
				   p80211msg_dot11req_mibset_t *msg,
				   void *data)
{
	int result;
	p80211pstrd_t *pstr = (p80211pstrd_t *) data;
	u8 bytebuf[MIB_TMP_MAXLEN];

	if (isget) {
		result =
		    hfa384x_drvr_getconfig(hw, mib->parm1, bytebuf, mib->parm2);
		prism2mgmt_bytearea2pstr(bytebuf, pstr, mib->parm2);
	} else {
		memset(bytebuf, 0, mib->parm2);
		prism2mgmt_pstr2bytearea(bytebuf, pstr);
		result =
		    hfa384x_drvr_setconfig(hw, mib->parm1, bytebuf, mib->parm2);
	}

	return result;
}


static int prism2mib_uint32(mibrec_t *mib,
			    int isget,
			    wlandevice_t *wlandev,
			    hfa384x_t *hw,
			    p80211msg_dot11req_mibset_t *msg, void *data)
{
	int result;
	u32 *uint32 = (u32 *) data;
	u8 bytebuf[MIB_TMP_MAXLEN];
	u16 *wordbuf = (u16 *) bytebuf;

	if (isget) {
		result = hfa384x_drvr_getconfig16(hw, mib->parm1, wordbuf);
		*uint32 = *wordbuf;
	} else {
		*wordbuf = *uint32;
		result = hfa384x_drvr_setconfig16(hw, mib->parm1, *wordbuf);
	}

	return result;
}


static int prism2mib_flag(mibrec_t *mib,
			  int isget,
			  wlandevice_t *wlandev,
			  hfa384x_t *hw,
			  p80211msg_dot11req_mibset_t *msg, void *data)
{
	int result;
	u32 *uint32 = (u32 *) data;
	u8 bytebuf[MIB_TMP_MAXLEN];
	u16 *wordbuf = (u16 *) bytebuf;
	u32 flags;

	result = hfa384x_drvr_getconfig16(hw, mib->parm1, wordbuf);
	if (result == 0) {
		flags = *wordbuf;
		if (isget) {
			*uint32 = (flags & mib->parm2) ?
			    P80211ENUM_truth_true : P80211ENUM_truth_false;
		} else {
			if ((*uint32) == P80211ENUM_truth_true)
				flags |= mib->parm2;
			else
				flags &= ~mib->parm2;
			*wordbuf = flags;
			result =
			    hfa384x_drvr_setconfig16(hw, mib->parm1, *wordbuf);
		}
	}

	return result;
}


static int prism2mib_wepdefaultkey(mibrec_t *mib,
				   int isget,
				   wlandevice_t *wlandev,
				   hfa384x_t *hw,
				   p80211msg_dot11req_mibset_t *msg,
				   void *data)
{
	int result;
	p80211pstrd_t *pstr = (p80211pstrd_t *) data;
	u8 bytebuf[MIB_TMP_MAXLEN];
	u16 len;

	if (isget) {
		result = 0;	/* Should never happen. */
	} else {
		len = (pstr->len > 5) ? HFA384x_RID_CNFWEP128DEFAULTKEY_LEN :
		    HFA384x_RID_CNFWEPDEFAULTKEY_LEN;
		memset(bytebuf, 0, len);
		prism2mgmt_pstr2bytearea(bytebuf, pstr);
		result = hfa384x_drvr_setconfig(hw, mib->parm1, bytebuf, len);
	}

	return result;
}


static int prism2mib_privacyinvoked(mibrec_t *mib,
				    int isget,
				    wlandevice_t *wlandev,
				    hfa384x_t *hw,
				    p80211msg_dot11req_mibset_t *msg,
				    void *data)
{
	int result;

	if (wlandev->hostwep & HOSTWEP_DECRYPT) {
		if (wlandev->hostwep & HOSTWEP_DECRYPT)
			mib->parm2 |= HFA384x_WEPFLAGS_DISABLE_RXCRYPT;
		if (wlandev->hostwep & HOSTWEP_ENCRYPT)
			mib->parm2 |= HFA384x_WEPFLAGS_DISABLE_TXCRYPT;
	}

	result = prism2mib_flag(mib, isget, wlandev, hw, msg, data);

	return result;
}


static int prism2mib_excludeunencrypted(mibrec_t *mib,
					int isget,
					wlandevice_t *wlandev,
					hfa384x_t *hw,
					p80211msg_dot11req_mibset_t *msg,
					void *data)
{
	int result;

	result = prism2mib_flag(mib, isget, wlandev, hw, msg, data);

	return result;
}


static int prism2mib_fragmentationthreshold(mibrec_t *mib,
					    int isget,
					    wlandevice_t *wlandev,
					    hfa384x_t *hw,
					    p80211msg_dot11req_mibset_t *msg,
					    void *data)
{
	int result;
	u32 *uint32 = (u32 *) data;

	if (!isget)
		if ((*uint32) % 2) {
			printk(KERN_WARNING "Attempt to set odd number "
			       "FragmentationThreshold\n");
			msg->resultcode.data =
			    P80211ENUM_resultcode_not_supported;
			return 0;
		}

	result = prism2mib_uint32(mib, isget, wlandev, hw, msg, data);

	return result;
}


static int prism2mib_priv(mibrec_t *mib,
			  int isget,
			  wlandevice_t *wlandev,
			  hfa384x_t *hw,
			  p80211msg_dot11req_mibset_t *msg, void *data)
{
	p80211pstrd_t *pstr = (p80211pstrd_t *) data;

	int result;

	switch (mib->did) {
	case DIDmib_lnx_lnxConfigTable_lnxRSNAIE:{
			hfa384x_WPAData_t wpa;
			if (isget) {
				hfa384x_drvr_getconfig(hw,
						       HFA384x_RID_CNFWPADATA,
						       (u8 *) &wpa,
						       sizeof(wpa));
				pstr->len = le16_to_cpu(wpa.datalen);
				memcpy(pstr->data, wpa.data, pstr->len);
			} else {
				wpa.datalen = cpu_to_le16(pstr->len);
				memcpy(wpa.data, pstr->data, pstr->len);

				result =
				    hfa384x_drvr_setconfig(hw,
						   HFA384x_RID_CNFWPADATA,
						   (u8 *) &wpa,
						   sizeof(wpa));
			}
			break;
		}
	default:
		printk(KERN_ERR "Unhandled DID 0x%08x\n", mib->did);
	}

	return 0;
}


void prism2mgmt_pstr2bytestr(hfa384x_bytestr_t *bytestr, p80211pstrd_t *pstr)
{
	bytestr->len = cpu_to_le16((u16) (pstr->len));
	memcpy(bytestr->data, pstr->data, pstr->len);
}


void prism2mgmt_pstr2bytearea(u8 *bytearea, p80211pstrd_t *pstr)
{
	memcpy(bytearea, pstr->data, pstr->len);
}


void prism2mgmt_bytestr2pstr(hfa384x_bytestr_t *bytestr, p80211pstrd_t *pstr)
{
	pstr->len = (u8) (le16_to_cpu((u16) (bytestr->len)));
	memcpy(pstr->data, bytestr->data, pstr->len);
}


void prism2mgmt_bytearea2pstr(u8 *bytearea, p80211pstrd_t *pstr, int len)
{
	pstr->len = (u8) len;
	memcpy(pstr->data, bytearea, len);
}

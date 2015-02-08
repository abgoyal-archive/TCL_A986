

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/wireless.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <net/sock.h>
#include <linux/netlink.h>

#include "p80211types.h"
#include "p80211hdr.h"
#include "p80211mgmt.h"
#include "p80211conv.h"
#include "p80211msg.h"
#include "p80211netdev.h"
#include "p80211ioctl.h"
#include "p80211metadef.h"
#include "p80211metastruct.h"
#include "p80211req.h"

static void p80211req_handlemsg(wlandevice_t *wlandev, p80211msg_t *msg);
static int p80211req_mibset_mibget(wlandevice_t *wlandev,
				   p80211msg_dot11req_mibget_t *mib_msg,
				   int isget);

int p80211req_dorequest(wlandevice_t *wlandev, u8 *msgbuf)
{
	int result = 0;
	p80211msg_t *msg = (p80211msg_t *) msgbuf;

	/* Check to make sure the MSD is running */
	if (!((wlandev->msdstate == WLAN_MSD_HWPRESENT &&
	       msg->msgcode == DIDmsg_lnxreq_ifstate) ||
	      wlandev->msdstate == WLAN_MSD_RUNNING ||
	      wlandev->msdstate == WLAN_MSD_FWLOAD)) {
		return -ENODEV;
	}

	/* Check Permissions */
	if (!capable(CAP_NET_ADMIN) &&
	(msg->msgcode != DIDmsg_dot11req_mibget)) {
		printk(KERN_ERR
		       "%s: only dot11req_mibget allowed for non-root.\n",
		       wlandev->name);
		return -EPERM;
	}

	/* Check for busy status */
	if (test_and_set_bit(1, &(wlandev->request_pending)))
		return -EBUSY;

	/* Allow p80211 to look at msg and handle if desired. */
	/* So far, all p80211 msgs are immediate, no waitq/timer necessary */
	/* This may change. */
	p80211req_handlemsg(wlandev, msg);

	/* Pass it down to wlandev via wlandev->mlmerequest */
	if (wlandev->mlmerequest != NULL)
		wlandev->mlmerequest(wlandev, msg);

	clear_bit(1, &(wlandev->request_pending));
	return result;	/* if result==0, msg->status still may contain an err */
}

static void p80211req_handlemsg(wlandevice_t *wlandev, p80211msg_t *msg)
{
	switch (msg->msgcode) {

	case DIDmsg_lnxreq_hostwep:{
			p80211msg_lnxreq_hostwep_t *req =
			    (p80211msg_lnxreq_hostwep_t *) msg;
			wlandev->hostwep &=
			    ~(HOSTWEP_DECRYPT | HOSTWEP_ENCRYPT);
			if (req->decrypt.data == P80211ENUM_truth_true)
				wlandev->hostwep |= HOSTWEP_DECRYPT;
			if (req->encrypt.data == P80211ENUM_truth_true)
				wlandev->hostwep |= HOSTWEP_ENCRYPT;

			break;
		}
	case DIDmsg_dot11req_mibget:
	case DIDmsg_dot11req_mibset:{
			int isget = (msg->msgcode == DIDmsg_dot11req_mibget);
			p80211msg_dot11req_mibget_t *mib_msg =
			    (p80211msg_dot11req_mibget_t *) msg;
			p80211req_mibset_mibget(wlandev, mib_msg, isget);
		}
	default:
		;
	}			/* switch msg->msgcode */

	return;
}

static int p80211req_mibset_mibget(wlandevice_t *wlandev,
				   p80211msg_dot11req_mibget_t *mib_msg,
				   int isget)
{
	p80211itemd_t *mibitem = (p80211itemd_t *) mib_msg->mibattribute.data;
	p80211pstrd_t *pstr = (p80211pstrd_t *) mibitem->data;
	u8 *key = mibitem->data + sizeof(p80211pstrd_t);

	switch (mibitem->did) {
	case DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey0:{
			if (!isget)
				wep_change_key(wlandev, 0, key, pstr->len);
			break;
		}
	case DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey1:{
			if (!isget)
				wep_change_key(wlandev, 1, key, pstr->len);
			break;
		}
	case DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey2:{
			if (!isget)
				wep_change_key(wlandev, 2, key, pstr->len);
			break;
		}
	case DIDmib_dot11smt_dot11WEPDefaultKeysTable_dot11WEPDefaultKey3:{
			if (!isget)
				wep_change_key(wlandev, 3, key, pstr->len);
			break;
		}
	case DIDmib_dot11smt_dot11PrivacyTable_dot11WEPDefaultKeyID:{
			u32 *data = (u32 *) mibitem->data;

			if (isget) {
				*data =
				    wlandev->hostwep & HOSTWEP_DEFAULTKEY_MASK;
			} else {
				wlandev->hostwep &= ~(HOSTWEP_DEFAULTKEY_MASK);

				wlandev->hostwep |=
				    (*data & HOSTWEP_DEFAULTKEY_MASK);
			}
			break;
		}
	case DIDmib_dot11smt_dot11PrivacyTable_dot11PrivacyInvoked:{
			u32 *data = (u32 *) mibitem->data;

			if (isget) {
				if (wlandev->hostwep & HOSTWEP_PRIVACYINVOKED)
					*data = P80211ENUM_truth_true;
				else
					*data = P80211ENUM_truth_false;
			} else {
				wlandev->hostwep &= ~(HOSTWEP_PRIVACYINVOKED);
				if (*data == P80211ENUM_truth_true)
					wlandev->hostwep |=
					    HOSTWEP_PRIVACYINVOKED;
			}
			break;
		}
	case DIDmib_dot11smt_dot11PrivacyTable_dot11ExcludeUnencrypted:{
			u32 *data = (u32 *) mibitem->data;

			if (isget) {
				if (wlandev->hostwep &
				    HOSTWEP_EXCLUDEUNENCRYPTED)
					*data = P80211ENUM_truth_true;
				else
					*data = P80211ENUM_truth_false;
			} else {
				wlandev->hostwep &=
				    ~(HOSTWEP_EXCLUDEUNENCRYPTED);
				if (*data == P80211ENUM_truth_true)
					wlandev->hostwep |=
					    HOSTWEP_EXCLUDEUNENCRYPTED;
			}
			break;
		}
	default:
		;
	}

	return 0;
}

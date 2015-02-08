


#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/socket.h>
#include <linux/audit.h>
#include <linux/tty.h>
#include <linux/security.h>
#include <linux/gfp.h>
#include <net/sock.h>
#include <net/netlink.h>
#include <net/genetlink.h>
#include <net/netlabel.h>
#include <asm/bug.h>

#include "netlabel_mgmt.h"
#include "netlabel_unlabeled.h"
#include "netlabel_cipso_v4.h"
#include "netlabel_user.h"


int __init netlbl_netlink_init(void)
{
	int ret_val;

	ret_val = netlbl_mgmt_genl_init();
	if (ret_val != 0)
		return ret_val;

	ret_val = netlbl_cipsov4_genl_init();
	if (ret_val != 0)
		return ret_val;

	ret_val = netlbl_unlabel_genl_init();
	if (ret_val != 0)
		return ret_val;

	return 0;
}


struct audit_buffer *netlbl_audit_start_common(int type,
					       struct netlbl_audit *audit_info)
{
	struct audit_buffer *audit_buf;
	char *secctx;
	u32 secctx_len;

	if (audit_enabled == 0)
		return NULL;

	audit_buf = audit_log_start(current->audit_context, GFP_ATOMIC, type);
	if (audit_buf == NULL)
		return NULL;

	audit_log_format(audit_buf, "netlabel: auid=%u ses=%u",
			 audit_info->loginuid,
			 audit_info->sessionid);

	if (audit_info->secid != 0 &&
	    security_secid_to_secctx(audit_info->secid,
				     &secctx,
				     &secctx_len) == 0) {
		audit_log_format(audit_buf, " subj=%s", secctx);
		security_release_secctx(secctx, secctx_len);
	}

	return audit_buf;
}



#include <mach/qdsp5/qdsp5vfemsg.h>
#include "adsp.h"

static int patch_op_event(struct msm_adsp_module *module,
				struct adsp_event *event)
{
	vfe_msg_op1 *op = (vfe_msg_op1 *)event->data.msg16;
	if (adsp_pmem_paddr_fixup(module, (void **)&op->op1_buf_y_addr) ||
	    adsp_pmem_paddr_fixup(module, (void **)&op->op1_buf_cbcr_addr))
		return -1;
	return 0;
}

static int patch_af_wb_event(struct msm_adsp_module *module,
				struct adsp_event *event)
{
	vfe_msg_stats_wb_exp *af = (vfe_msg_stats_wb_exp *)event->data.msg16;
	return adsp_pmem_paddr_fixup(module, (void **)&af->wb_exp_stats_op_buf);
}

int adsp_vfe_patch_event(struct msm_adsp_module *module,
			struct adsp_event *event)
{
	switch(event->msg_id) {
	case VFE_MSG_OP1:
	case VFE_MSG_OP2:
		return patch_op_event(module, event);
	case VFE_MSG_STATS_AF:
	case VFE_MSG_STATS_WB_EXP:
		return patch_af_wb_event(module, event);
	default:
		break;
	}

	return 0;
}

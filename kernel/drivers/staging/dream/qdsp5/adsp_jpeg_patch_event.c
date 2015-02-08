

#include <mach/qdsp5/qdsp5jpegmsg.h>
#include "adsp.h"

int adsp_jpeg_patch_event(struct msm_adsp_module *module,
			struct adsp_event *event)
{
	if (event->msg_id == JPEG_MSG_ENC_OP_PRODUCED) {
		jpeg_msg_enc_op_produced *op = (jpeg_msg_enc_op_produced *)event->data.msg16;
		return adsp_pmem_paddr_fixup(module, (void **)&op->op_buf_addr);
	}

	return 0;
}

#include "globals.h"

static inline void cvt_pdu_remap(struct blk_io_trace_remap *rp)
{
	rp->device_from = be32_to_cpu(rp->device_from);
	rp->device_to   = be32_to_cpu(rp->device_to);
	rp->sector_from = be64_to_cpu(rp->sector_from);
}

void trace_remap(struct io *a_iop)
{
	struct io *q_iop;
	struct d_info *q_dip;
	struct blk_io_trace_remap *rp;

	if (ignore_remaps)
		goto out;

	rp = a_iop->pdu;
	cvt_pdu_remap(rp);

	if (!io_setup(a_iop, IOP_A))
		goto out;

	q_dip = __dip_find(rp->device_from);
	if (!q_dip)
		goto out;

	q_iop = dip_find_sec(q_dip, IOP_Q, rp->sector_from);
	if (q_iop)
		update_q2a(q_iop, tdelta(q_iop->t.time, a_iop->t.time));

out:
	io_release(a_iop);
}

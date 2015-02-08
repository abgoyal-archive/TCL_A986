#include "globals.h"

static __u64 get_nio_up(struct io *u_iop)
{
	__u64 *val = u_iop->pdu;
	return be64_to_cpu(*val);
}

void trace_unplug_io(struct io *u_iop)
{
	unplug_hist_add(u_iop);
	dip_unplug(u_iop->t.device, BIT_TIME(u_iop->t.time), get_nio_up(u_iop));
	io_release(u_iop);
}

void trace_unplug_timer(struct io *ut_iop)
{
	dip_unplug_tm(ut_iop->t.device, BIT_TIME(ut_iop->t.time),
			get_nio_up(ut_iop));
	io_release(ut_iop);
}

void trace_plug(struct io *p_iop)
{
	dip_plug(p_iop->t.device, BIT_TIME(p_iop->t.time));
	io_release(p_iop);
}

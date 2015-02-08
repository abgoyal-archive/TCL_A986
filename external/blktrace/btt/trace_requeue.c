#include "globals.h"

static void handle_requeue(struct io *r_iop)
{
	r_iop->dip->n_qs++;
	r_iop->dip->t_act_q += r_iop->dip->n_act_q;
	r_iop->dip->n_act_q++;
}

void trace_requeue(struct io *r_iop)
{
	if (io_setup(r_iop, IOP_R))
		handle_requeue(r_iop);
	io_release(r_iop);
}

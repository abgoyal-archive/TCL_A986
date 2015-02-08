

#ifndef __HIPZ_FNS_CORE_H__
#define __HIPZ_FNS_CORE_H__

#include "hcp_phyp.h"
#include "hipz_hw.h"

#define hipz_galpa_store_cq(gal, offset, value) \
	hipz_galpa_store(gal, CQTEMM_OFFSET(offset), value)

#define hipz_galpa_load_cq(gal, offset) \
	hipz_galpa_load(gal, CQTEMM_OFFSET(offset))

#define hipz_galpa_store_qp(gal, offset, value) \
	hipz_galpa_store(gal, QPTEMM_OFFSET(offset), value)
#define hipz_galpa_load_qp(gal, offset) \
	hipz_galpa_load(gal, QPTEMM_OFFSET(offset))

static inline void hipz_update_sqa(struct ehca_qp *qp, u16 nr_wqes)
{
	/*  ringing doorbell :-) */
	hipz_galpa_store_qp(qp->galpas.kernel, qpx_sqa,
			    EHCA_BMASK_SET(QPX_SQADDER, nr_wqes));
}

static inline void hipz_update_rqa(struct ehca_qp *qp, u16 nr_wqes)
{
	/*  ringing doorbell :-) */
	hipz_galpa_store_qp(qp->galpas.kernel, qpx_rqa,
			    EHCA_BMASK_SET(QPX_RQADDER, nr_wqes));
}

static inline void hipz_update_feca(struct ehca_cq *cq, u32 nr_cqes)
{
	hipz_galpa_store_cq(cq->galpas.kernel, cqx_feca,
			    EHCA_BMASK_SET(CQX_FECADDER, nr_cqes));
}

static inline void hipz_set_cqx_n0(struct ehca_cq *cq, u32 value)
{
	u64 cqx_n0_reg;

	hipz_galpa_store_cq(cq->galpas.kernel, cqx_n0,
			    EHCA_BMASK_SET(CQX_N0_GENERATE_SOLICITED_COMP_EVENT,
					   value));
	cqx_n0_reg = hipz_galpa_load_cq(cq->galpas.kernel, cqx_n0);
}

static inline void hipz_set_cqx_n1(struct ehca_cq *cq, u32 value)
{
	u64 cqx_n1_reg;

	hipz_galpa_store_cq(cq->galpas.kernel, cqx_n1,
			    EHCA_BMASK_SET(CQX_N1_GENERATE_COMP_EVENT, value));
	cqx_n1_reg = hipz_galpa_load_cq(cq->galpas.kernel, cqx_n1);
}

#endif /* __HIPZ_FNC_CORE_H__ */



#ifndef MTHCA_PROFILE_H
#define MTHCA_PROFILE_H

#include "mthca_dev.h"
#include "mthca_cmd.h"

struct mthca_profile {
	int num_qp;
	int rdb_per_qp;
	int num_srq;
	int num_cq;
	int num_mcg;
	int num_mpt;
	int num_mtt;
	int num_udav;
	int num_uar;
	int uarc_size;
	int fmr_reserved_mtts;
};

s64 mthca_make_profile(struct mthca_dev *mdev,
		       struct mthca_profile *request,
		       struct mthca_dev_lim *dev_lim,
		       struct mthca_init_hca_param *init_hca);

#endif /* MTHCA_PROFILE_H */

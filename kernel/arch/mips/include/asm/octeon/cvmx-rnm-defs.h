

#ifndef __CVMX_RNM_DEFS_H__
#define __CVMX_RNM_DEFS_H__

#include <linux/types.h>

#define CVMX_RNM_BIST_STATUS \
	 CVMX_ADD_IO_SEG(0x0001180040000008ull)
#define CVMX_RNM_CTL_STATUS \
	 CVMX_ADD_IO_SEG(0x0001180040000000ull)

union cvmx_rnm_bist_status {
	uint64_t u64;
	struct cvmx_rnm_bist_status_s {
		uint64_t reserved_2_63:62;
		uint64_t rrc:1;
		uint64_t mem:1;
	} s;
	struct cvmx_rnm_bist_status_s cn30xx;
	struct cvmx_rnm_bist_status_s cn31xx;
	struct cvmx_rnm_bist_status_s cn38xx;
	struct cvmx_rnm_bist_status_s cn38xxp2;
	struct cvmx_rnm_bist_status_s cn50xx;
	struct cvmx_rnm_bist_status_s cn52xx;
	struct cvmx_rnm_bist_status_s cn52xxp1;
	struct cvmx_rnm_bist_status_s cn56xx;
	struct cvmx_rnm_bist_status_s cn56xxp1;
	struct cvmx_rnm_bist_status_s cn58xx;
	struct cvmx_rnm_bist_status_s cn58xxp1;
};

union cvmx_rnm_ctl_status {
	uint64_t u64;
	struct cvmx_rnm_ctl_status_s {
		uint64_t reserved_9_63:55;
		uint64_t ent_sel:4;
		uint64_t exp_ent:1;
		uint64_t rng_rst:1;
		uint64_t rnm_rst:1;
		uint64_t rng_en:1;
		uint64_t ent_en:1;
	} s;
	struct cvmx_rnm_ctl_status_cn30xx {
		uint64_t reserved_4_63:60;
		uint64_t rng_rst:1;
		uint64_t rnm_rst:1;
		uint64_t rng_en:1;
		uint64_t ent_en:1;
	} cn30xx;
	struct cvmx_rnm_ctl_status_cn30xx cn31xx;
	struct cvmx_rnm_ctl_status_cn30xx cn38xx;
	struct cvmx_rnm_ctl_status_cn30xx cn38xxp2;
	struct cvmx_rnm_ctl_status_s cn50xx;
	struct cvmx_rnm_ctl_status_s cn52xx;
	struct cvmx_rnm_ctl_status_s cn52xxp1;
	struct cvmx_rnm_ctl_status_s cn56xx;
	struct cvmx_rnm_ctl_status_s cn56xxp1;
	struct cvmx_rnm_ctl_status_s cn58xx;
	struct cvmx_rnm_ctl_status_s cn58xxp1;
};

#endif

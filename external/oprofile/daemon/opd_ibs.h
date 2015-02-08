
#ifndef OPD_IBS_H
#define OPD_IBS_H

#include <stdint.h>

#include "opd_ibs_macro.h"

struct transient;
struct opd_event;


struct ibs_sample {
	struct ibs_fetch_sample * fetch;
	struct ibs_op_sample * op;
};

struct ibs_fetch_sample {
	unsigned long int rip;
	/* MSRC001_1030 IBS Fetch Control Register */
	unsigned int ibs_fetch_ctl_low;
	unsigned int ibs_fetch_ctl_high;
	/* MSRC001_1031 IBS Fetch Linear Address Register */
	unsigned int ibs_fetch_lin_addr_low;
	unsigned int ibs_fetch_lin_addr_high;
	/* MSRC001_1032 IBS Fetch Physical Address Register */
	unsigned int ibs_fetch_phys_addr_low;
	unsigned int ibs_fetch_phys_addr_high;
	unsigned int dummy_event;
};



/** This struct represents the hardware-level IBS op information. */
struct ibs_op_sample {
	unsigned long int rip;
	/* MSRC001_1034 IBS Op Logical Address Register */
	unsigned int ibs_op_lin_addr_low;
	unsigned int ibs_op_lin_addr_high;
	/* MSRC001_1035 IBS Op Data Register */
	unsigned int ibs_op_data1_low;
	unsigned int ibs_op_data1_high;
	/* MSRC001_1036 IBS Op Data 2 Register */
	unsigned int ibs_op_data2_low;
	unsigned int ibs_op_data2_high;
	/* MSRC001_1037 IBS Op Data 3 Register */
	unsigned int ibs_op_data3_low;
	unsigned int ibs_op_data3_high;
	unsigned int ibs_op_ldst_linaddr_low;
	unsigned int ibs_op_ldst_linaddr_high;
	unsigned int ibs_op_phys_addr_low;
	unsigned int ibs_op_phys_addr_high;
};


enum IBSL1PAGESIZE {
	L1TLB4K = 0,
	L1TLB2M,
	L1TLB1G,
	L1TLB_INVALID
};


extern void code_ibs_fetch_sample(struct transient * trans);

extern void code_ibs_op_sample(struct transient * trans);

/** Log the specified IBS derived event. */
extern void opd_log_ibs_event(unsigned int event, struct transient * trans);

/** Log the specified IBS cycle count. */
extern void opd_log_ibs_count(unsigned int event, struct transient * trans, unsigned int count);


#endif /*OPD_IBS_H*/

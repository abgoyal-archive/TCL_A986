
#if !defined(__BTRECORD_H__)
#define __BTRECORD_H__

#include <asm/types.h>

#define BT_MAX_PKTS	512

struct io_bunch_hdr {
	__u64 npkts;
	__u64 time_stamp;
};

struct io_pkt {
	__u64 sector;
	__u64 nbytes;
	__u32 rw;
};

struct io_bunch {
	struct io_bunch_hdr hdr;
	struct io_pkt pkts[BT_MAX_PKTS];
};

struct io_file_hdr {
	__u64 version;
	__u64 genesis;
	__u64 nbunches;
	__u64 total_pkts;
};

static inline __u64 mk_btversion(int mjr, int mnr, int sub)
{
	return ((mjr & 0xff) << 16) | ((mnr & 0xff) << 8) | (sub & 0xff);
}

static inline void get_btversion(__u64 version, int *mjr, int *mnr, int *sub)
{
	*mjr = (int)((version >> 16) & 0xff);
	*mnr = (int)((version >>  8) & 0xff);
	*sub = (int)((version >>  0) & 0xff);
}

static char my_btversion[] = "1.0.0";
static int btver_mjr = 1;
static int btver_mnr = 0;
static int btver_sub = 0;

#endif

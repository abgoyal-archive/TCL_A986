#include "globals.h"

static inline void latency_out(FILE *ofp, __u64 tstamp, __u64 latency)
{
	if (ofp)
		fprintf(ofp, "%lf %lf\n", TO_SEC(tstamp), TO_SEC(latency));
}

FILE *latency_open(__u32 device, char *name, char *post)
{
	FILE *fp = NULL;

	if (name) {
		int mjr, mnr;
		char oname[strlen(name) + 32];

		mjr = device >> MINORBITS;
		mnr = device & ((1 << MINORBITS) - 1);

		sprintf(oname, "%s_%03d,%03d_%s.dat", name, mjr, mnr, post);
		if ((fp = my_fopen(oname, "w")) == NULL)
			perror(oname);
		else
			add_file(fp, strdup(oname));
	}

	return fp;
}

void latency_alloc(struct d_info *dip)
{
	dip->q2d_ofp = latency_open(dip->device, q2d_name, "q2d");
	dip->d2c_ofp = latency_open(dip->device, d2c_name, "d2c");
	dip->q2c_ofp = latency_open(dip->device, q2c_name, "q2c");
}

void latency_q2d(struct d_info *dip, __u64 tstamp, __u64 latency)
{
	plat_x2c(dip->q2d_plat_handle, tstamp, latency);
	latency_out(dip->q2d_ofp, tstamp, latency);
}

void latency_d2c(struct d_info *dip, __u64 tstamp, __u64 latency)
{
	plat_x2c(dip->d2c_plat_handle, tstamp, latency);
	latency_out(dip->d2c_ofp, tstamp, latency);
}

void latency_q2c(struct d_info *dip, __u64 tstamp, __u64 latency)
{
	plat_x2c(dip->q2c_plat_handle, tstamp, latency);
	latency_out(dip->q2c_ofp, tstamp, latency);
}

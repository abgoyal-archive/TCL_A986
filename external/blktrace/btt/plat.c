#include "globals.h"

struct plat_info {
	long nl;
	FILE *fp;
	double first_ts, last_ts, tl;
};

void *plat_alloc(char *str)
{
	char *oname;
	struct plat_info *pp;

	if (plat_freq <= 0.0) return NULL;

	pp = malloc(sizeof(*pp));
	pp->nl = 0;
	pp->first_ts = pp->last_ts = pp->tl = -1.0;

	oname = malloc(strlen(str) + 32);
	sprintf(oname, "%s.dat", str);
	if ((pp->fp = my_fopen(oname, "w")) == NULL) {
		perror(oname);
		return NULL;
	}
	add_file(pp->fp, oname);

	return pp;
}

void plat_free(void *info)
{
	struct plat_info *pp = info;

	if (pp == NULL) return;

	if (pp->first_ts != -1.0) {
		double delta = pp->last_ts - pp->first_ts;

		fprintf(pp->fp, "%lf %lf\n",
			pp->first_ts + (delta / 2), pp->tl / pp->nl);
	}
	free(info);
}

void plat_x2c(void *info, __u64 ts, __u64 latency)
{
	double now = TO_SEC(ts);
	double lat = TO_SEC(latency);
	struct plat_info *pp = info;

	if (pp == NULL) return;

	if (pp->first_ts == -1.0) {
		pp->first_ts = pp->last_ts = now;
		pp->nl = 1;
		pp->tl = lat;
	} else if ((now - pp->first_ts) >= plat_freq) {
		double delta = pp->last_ts - pp->first_ts;

		fprintf(pp->fp, "%lf %lf\n",
			pp->first_ts + (delta / 2), pp->tl / pp->nl);

		pp->first_ts = pp->last_ts = now;
		pp->nl = 1;
		pp->tl = lat;
	} else {
		pp->last_ts = now;
		pp->nl += 1;
		pp->tl += lat;
	}
}

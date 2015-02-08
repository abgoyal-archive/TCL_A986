#include "globals.h"

struct bno_dump {
	FILE *rfp, *wfp, *cfp;
};

static FILE *bno_dump_open(__u32 device, char rwc)
{
	FILE *fp;
	char *oname;
	int mjr, mnr;

	mjr = device >> MINORBITS;
	mnr = device & ((1 << MINORBITS) - 1);

	oname = malloc(strlen(bno_dump_name) + 32);
	sprintf(oname, "%s_%03d,%03d_%c.dat", bno_dump_name, mjr, mnr, rwc);
	if ((fp = my_fopen(oname, "w")) == NULL)
		perror(oname);
	else
		add_file(fp, oname);
	return fp;
}

static inline void bno_dump_write(FILE *fp, struct io *iop)
{
	fprintf(fp, "%15.9lf %lld %lld\n", BIT_TIME(iop->t.time),
		(long long)BIT_START(iop), (long long)BIT_END(iop));
}

void *bno_dump_alloc(__u32 device)
{
	struct bno_dump *bdp;

	if (bno_dump_name == NULL) return NULL;

	bdp = malloc(sizeof(*bdp));
	bdp->rfp = bno_dump_open(device, 'r');
	bdp->wfp = bno_dump_open(device, 'w');
	bdp->cfp = bno_dump_open(device, 'c');

	return bdp;
}

void bno_dump_free(void *param)
{
	free(param);
}

void bno_dump_add(void *handle, struct io *iop)
{
	struct bno_dump *bdp = handle;

	if (bdp) {
		FILE *fp = IOP_READ(iop) ? bdp->rfp : bdp->wfp;

		if (fp)
			bno_dump_write(fp, iop);
		if (bdp->cfp)
			bno_dump_write(bdp->cfp, iop);
	}
}

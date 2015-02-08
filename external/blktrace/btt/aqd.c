#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "globals.h"

struct aqd_info {
	FILE *fp;
	int na;		/* # active */
};

void *aqd_alloc(char *str)
{
	char *oname;
	struct aqd_info *ap;

	if (aqd_name == NULL) return NULL;

	ap = malloc(sizeof(*ap));
	ap->na = 0;

	oname = malloc(strlen(aqd_name) + strlen(str) + 32);
	sprintf(oname, "%s_%s.dat", aqd_name, str);
	if ((ap->fp = my_fopen(oname, "w")) == NULL) {
		perror(oname);
		return NULL;
	}
	add_file(ap->fp, oname);

	return ap;

}

void aqd_free(void *info)
{
	free(info);
}

void aqd_issue(void *info, double ts)
{
	if (info) {
		struct aqd_info *ap = info;

		fprintf(ap->fp, "%lf %d\n%lf %d\n", ts, ap->na, ts, ap->na + 1);
		ap->na += 1;
	}
}

void aqd_complete(void *info, double ts)
{
	if (info) {
		struct aqd_info *ap = info;

		if (ap->na > 0) {
			fprintf(ap->fp, "%lf %d\n%lf %d\n",
					ts, ap->na, ts, ap->na - 1);
			ap->na -= 1;
		}
	}
}

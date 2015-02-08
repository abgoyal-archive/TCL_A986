

static const unsigned char sn9c2028_sof_marker[5] =
	{ 0xff, 0xff, 0x00, 0xc4, 0xc4 };

static unsigned char *sn9c2028_find_sof(struct gspca_dev *gspca_dev,
					unsigned char *m, int len)
{
	struct sd *sd = (struct sd *) gspca_dev;
	int i;

	/* Search for the SOF marker (fixed part) in the header */
	for (i = 0; i < len; i++) {
		if (m[i] == sn9c2028_sof_marker[sd->sof_read]) {
			sd->sof_read++;
			if (sd->sof_read == sizeof(sn9c2028_sof_marker)) {
				PDEBUG(D_FRAM,
					"SOF found, bytes to analyze: %u."
					" Frame starts at byte #%u",
					len, i + 1);
				sd->sof_read = 0;
				return m + i + 1;
			}
		} else {
			sd->sof_read = 0;
		}
	}

	return NULL;
}

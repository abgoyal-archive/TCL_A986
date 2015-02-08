
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <stdint.h>

#include "csr.h"

static uint16_t seqnum = 0x0000;

int csr_open_3wire(char *device)
{
	fprintf(stderr, "Transport not implemented\n");

	return -1;
}

static int do_command(uint16_t command, uint16_t seqnum, uint16_t varid, uint8_t *value, uint16_t length)
{
	errno = EIO;

	return -1;
}

int csr_read_3wire(uint16_t varid, uint8_t *value, uint16_t length)
{
	return do_command(0x0000, seqnum++, varid, value, length);
}

int csr_write_3wire(uint16_t varid, uint8_t *value, uint16_t length)
{
	return do_command(0x0002, seqnum++, varid, value, length);
}

void csr_close_3wire(void)
{
}

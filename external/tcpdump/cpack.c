

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <string.h>
#include <tcpdump-stdinc.h>

#include "cpack.h"
#include "extract.h"

static u_int8_t *
cpack_next_boundary(u_int8_t *buf, u_int8_t *p, size_t alignment)
{
	size_t misalignment = (size_t)(p - buf) % alignment;

	if (misalignment == 0)
		return p;

	return p + (alignment - misalignment);
}

static u_int8_t *
cpack_align_and_reserve(struct cpack_state *cs, size_t wordsize)
{
	u_int8_t *next;

	/* Ensure alignment. */
	next = cpack_next_boundary(cs->c_buf, cs->c_next, wordsize);

	/* Too little space for wordsize bytes? */
	if (next - cs->c_buf + wordsize > cs->c_len)
		return NULL;

	return next;
}

int
cpack_init(struct cpack_state *cs, u_int8_t *buf, size_t buflen)
{
	memset(cs, 0, sizeof(*cs));

	cs->c_buf = buf;
	cs->c_len = buflen;
	cs->c_next = cs->c_buf;

	return 0;
}

/* Unpack a 64-bit unsigned integer. */
int
cpack_uint64(struct cpack_state *cs, u_int64_t *u)
{
	u_int8_t *next;

	if ((next = cpack_align_and_reserve(cs, sizeof(*u))) == NULL)
		return -1;

	*u = EXTRACT_LE_64BITS(next);

	/* Move pointer past the u_int64_t. */
	cs->c_next = next + sizeof(*u);
	return 0;
}

/* Unpack a 32-bit unsigned integer. */
int
cpack_uint32(struct cpack_state *cs, u_int32_t *u)
{
	u_int8_t *next;

	if ((next = cpack_align_and_reserve(cs, sizeof(*u))) == NULL)
		return -1;

	*u = EXTRACT_LE_32BITS(next);

	/* Move pointer past the u_int32_t. */
	cs->c_next = next + sizeof(*u);
	return 0;
}

/* Unpack a 16-bit unsigned integer. */
int
cpack_uint16(struct cpack_state *cs, u_int16_t *u)
{
	u_int8_t *next;

	if ((next = cpack_align_and_reserve(cs, sizeof(*u))) == NULL)
		return -1;

	*u = EXTRACT_LE_16BITS(next);

	/* Move pointer past the u_int16_t. */
	cs->c_next = next + sizeof(*u);
	return 0;
}

/* Unpack an 8-bit unsigned integer. */
int
cpack_uint8(struct cpack_state *cs, u_int8_t *u)
{
	/* No space left? */
	if ((size_t)(cs->c_next - cs->c_buf) >= cs->c_len)
		return -1;

	*u = *cs->c_next;

	/* Move pointer past the u_int8_t. */
	cs->c_next++;
	return 0;
}

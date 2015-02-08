
/* Contains helper functions for mp_int handling */

#include "includes.h"
#include "dbutil.h"

/* wrapper for mp_init, failing fatally on errors (memory allocation) */
void m_mp_init(mp_int *mp) {

	if (mp_init(mp) != MP_OKAY) {
		dropbear_exit("mem alloc error");
	}
}

void m_mp_init_multi(mp_int *mp, ...) 
{
    mp_int* cur_arg = mp;
    va_list args;

    va_start(args, mp);        /* init args to next argument from caller */
    while (cur_arg != NULL) {
        if (mp_init(cur_arg) != MP_OKAY) {
			dropbear_exit("mem alloc error");
        }
        cur_arg = va_arg(args, mp_int*);
    }
    va_end(args);
}

void bytes_to_mp(mp_int *mp, const unsigned char* bytes, unsigned int len) {

	if (mp_read_unsigned_bin(mp, (unsigned char*)bytes, len) != MP_OKAY) {
		dropbear_exit("mem alloc error");
	}
}

/* hash the ssh representation of the mp_int mp */
void sha1_process_mp(hash_state *hs, mp_int *mp) {

	int i;
	buffer * buf;

	buf = buf_new(512 + 20); /* max buffer is a 4096 bit key, 
								plus header + some leeway*/
	buf_putmpint(buf, mp);
	i = buf->pos;
	buf_setpos(buf, 0);
	sha1_process(hs, buf_getptr(buf, i), i);
	buf_free(buf);
}

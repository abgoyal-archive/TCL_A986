
#include "includes.h"
#include "runopts.h"
#include "signkey.h"
#include "buffer.h"
#include "dbutil.h"
#include "auth.h"

runopts opts; /* GLOBAL */

int readhostkey(const char * filename, sign_key * hostkey, int *type) {

	int ret = DROPBEAR_FAILURE;
	buffer *buf;

	buf = buf_new(MAX_PRIVKEY_SIZE);

	if (buf_readfile(buf, filename) == DROPBEAR_FAILURE) {
		goto out;
	}
	buf_setpos(buf, 0);
	if (buf_get_priv_key(buf, hostkey, type) == DROPBEAR_FAILURE) {
		goto out;
	}

	ret = DROPBEAR_SUCCESS;
out:

	buf_burn(buf);
	buf_free(buf);
	return ret;
}

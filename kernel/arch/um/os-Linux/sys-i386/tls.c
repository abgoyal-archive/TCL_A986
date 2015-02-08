
#include <errno.h>
#include <linux/unistd.h>

#include <sys/syscall.h>
#include <unistd.h>

#include "sysdep/tls.h"
#include "user.h"

void check_host_supports_tls(int *supports_tls, int *tls_min) {
	/* Values for x86 and x86_64.*/
	int val[] = {GDT_ENTRY_TLS_MIN_I386, GDT_ENTRY_TLS_MIN_X86_64};
	int i;

	for (i = 0; i < ARRAY_SIZE(val); i++) {
		user_desc_t info;
		info.entry_number = val[i];

		if (syscall(__NR_get_thread_area, &info) == 0) {
			*tls_min = val[i];
			*supports_tls = 1;
			return;
		} else {
			if (errno == EINVAL)
				continue;
			else if (errno == ENOSYS)
				*supports_tls = 0;
				return;
		}
	}

	*supports_tls = 0;
}

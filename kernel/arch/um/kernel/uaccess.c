


#include <linux/string.h>
#include "os.h"

static void __do_copy(void *to, const void *from, int n)
{
	memcpy(to, from, n);
}


int __do_copy_to_user(void *to, const void *from, int n,
		      void **fault_addr, jmp_buf **fault_catcher)
{
	unsigned long fault;
	int faulted;

	fault = __do_user_copy(to, from, n, fault_addr, fault_catcher,
			       __do_copy, &faulted);
	if (!faulted)
		return 0;
	else
		return n - (fault - (unsigned long) to);
}

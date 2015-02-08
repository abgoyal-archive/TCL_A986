
#include <linux/kernel.h>
#include <linux/rwsem.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/debug_locks.h>

int debug_locks = 1;
EXPORT_SYMBOL_GPL(debug_locks);

int debug_locks_silent;

int debug_locks_off(void)
{
	if (__debug_locks_off()) {
		if (!debug_locks_silent) {
			oops_in_progress = 1;
			console_verbose();
			return 1;
		}
	}
	return 0;
}

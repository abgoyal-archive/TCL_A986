

#include <cs/bfa_debug.h>
#include <bfa_os_inc.h>
#include <cs/bfa_q.h>
#include <log/bfa_log_hal.h>



void
bfa_panic(int line, char *file, char *panicstr)
{
	bfa_log(NULL, BFA_LOG_HAL_ASSERT, file, line, panicstr);
	bfa_os_panic();
}

void
bfa_sm_panic(struct bfa_log_mod_s *logm, int line, char *file, int event)
{
	bfa_log(logm, BFA_LOG_HAL_SM_ASSERT, file, line, event);
	bfa_os_panic();
}

int
bfa_q_is_on_q_func(struct list_head *q, struct list_head *qe)
{
	struct list_head        *tqe;

	tqe = bfa_q_next(q);
	while (tqe != q) {
		if (tqe == qe)
			return 1;
		tqe = bfa_q_next(tqe);
		if (tqe == NULL)
			break;
	}
	return 0;
}



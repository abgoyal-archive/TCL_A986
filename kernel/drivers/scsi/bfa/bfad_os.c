


#include "bfa_os_inc.h"
#include "bfad_drv.h"

void
bfa_os_gettimeofday(struct bfa_timeval_s *tv)
{
	struct timeval  tmp_tv;

	do_gettimeofday(&tmp_tv);
	tv->tv_sec = (u32) tmp_tv.tv_sec;
	tv->tv_usec = (u32) tmp_tv.tv_usec;
}

void
bfa_os_printf(struct bfa_log_mod_s *log_mod, u32 msg_id,
			const char *fmt, ...)
{
	va_list ap;
	#define BFA_STRING_256	256
	char tmp[BFA_STRING_256];

	va_start(ap, fmt);
	vsprintf(tmp, fmt, ap);
	va_end(ap);

	printk(tmp);
}



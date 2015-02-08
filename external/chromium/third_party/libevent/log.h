#ifndef _LOG_H_
#define _LOG_H_

#ifdef __GNUC__
#define EV_CHECK_FMT(a,b) __attribute__((format(printf, a, b)))
#else
#define EV_CHECK_FMT(a,b)
#endif

void event_err(int eval, const char *fmt, ...) EV_CHECK_FMT(2,3);
void event_warn(const char *fmt, ...) EV_CHECK_FMT(1,2);
void event_errx(int eval, const char *fmt, ...) EV_CHECK_FMT(2,3);
void event_warnx(const char *fmt, ...) EV_CHECK_FMT(1,2);
void event_msgx(const char *fmt, ...) EV_CHECK_FMT(1,2);
void _event_debugx(const char *fmt, ...) EV_CHECK_FMT(1,2);

#ifdef USE_DEBUG
#define event_debug(x) _event_debugx x
#else
#define event_debug(x) do {;} while (0)
#endif

#undef EV_CHECK_FMT

#endif

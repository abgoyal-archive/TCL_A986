

/* These functions are not used anymore, but need to stay for binary compatibility <rdar://problem/4841432>. */
/* You should use <wtf/Assertions.h>. */

void WebReportAssertionFailure(const char *file, int line, const char *function, const char *assertion);
void WebReportError(const char *file, int line, const char *function, const char *format, ...);

void WebReportAssertionFailure(const char *file, int line, const char *function, const char *assertion)
{
}

void WebReportError(const char *file, int line, const char *function, const char *format, ...)
{
}


#include <stdio.h>
#include "com_err.h"
#include "error_table.h"
#include "internal.h"

static void
default_com_err_proc (const char *whoami, errcode_t code, const
		      char *fmt, va_list args)
	COM_ERR_ATTR((format(printf, 3, 0)));

static void
default_com_err_proc (const char *whoami, errcode_t code, const
		      char *fmt, va_list args)
{
    if (whoami) {
	fputs(whoami, stderr);
	fputs(": ", stderr);
    }
    if (code) {
	fputs(error_message(code), stderr);
	fputs(" ", stderr);
    }
    if (fmt) {
        vfprintf (stderr, fmt, args);
    }
    /* should output \r only if using a tty in raw mode */
    fputs("\r\n", stderr);
    fflush(stderr);
}

typedef void (*errf) (const char *, errcode_t, const char *, va_list);

errf com_err_hook = default_com_err_proc;

void com_err_va (const char *whoami, errcode_t code, const char *fmt,
		 va_list args)
{
    (*com_err_hook) (whoami, code, fmt, args);
}

void com_err (const char *whoami,
	      errcode_t code,
	      const char *fmt, ...)
{
    va_list pvar;

    if (!com_err_hook)
	com_err_hook = default_com_err_proc;
    va_start(pvar, fmt);
    com_err_va (whoami, code, fmt, pvar);
    va_end(pvar);
}

errf set_com_err_hook (new_proc)
    errf new_proc;
{
    errf x = com_err_hook;

    if (new_proc)
	com_err_hook = new_proc;
    else
	com_err_hook = default_com_err_proc;

    return x;
}

errf reset_com_err_hook () {
    errf x = com_err_hook;
    com_err_hook = default_com_err_proc;
    return x;
}

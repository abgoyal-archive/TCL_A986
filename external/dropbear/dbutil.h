
#ifndef _DBUTIL_H_

#define _DBUTIL_H_

#include "includes.h"
#include "buffer.h"

#ifndef DISABLE_SYSLOG
void startsyslog();
#endif

extern void (*_dropbear_exit)(int exitcode, const char* format, va_list param);
extern void (*_dropbear_log)(int priority, const char* format, va_list param);

void dropbear_exit(const char* format, ...);
void dropbear_close(const char* format, ...);
void dropbear_log(int priority, const char* format, ...);
void fail_assert(const char* expr, const char* file, int line);
#ifdef DEBUG_TRACE
void dropbear_trace(const char* format, ...);
void printhex(const char * label, const unsigned char * buf, int len);
extern int debug_trace;
#endif
char * stripcontrol(const char * text);
unsigned char * getaddrstring(struct sockaddr_storage* addr, int withport);
int dropbear_listen(const char* address, const char* port,
		int *socks, unsigned int sockcount, char **errstring, int *maxfd);
int connect_remote(const char* remotehost, const char* remoteport,
		int nonblocking, char ** errstring);
char* getaddrhostname(struct sockaddr_storage * addr);
int buf_readfile(buffer* buf, const char* filename);
int buf_getline(buffer * line, FILE * authfile);

void m_close(int fd);
void * m_malloc(size_t size);
void * m_strdup(const char * str);
void * m_realloc(void* ptr, size_t size);
#define m_free(X) __m_free(X); (X) = NULL;
void __m_free(void* ptr);
void m_burn(void* data, unsigned int len);
void setnonblocking(int fd);
void disallow_core();

/* Used to force mp_ints to be initialised */
#define DEF_MP_INT(X) mp_int X = {0, 0, 0, NULL}

/* Dropbear assertion */
#define dropbear_assert(X) do { if (!(X)) { fail_assert(#X, __FILE__, __LINE__); } } while (0)

#endif /* _DBUTIL_H_ */

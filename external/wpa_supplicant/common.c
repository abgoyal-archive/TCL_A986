
#include "includes.h"

#include "common.h"


#ifdef CONFIG_DEBUG_FILE
static FILE *out_file = NULL;
#endif /* CONFIG_DEBUG_FILE */
int wpa_debug_level = MSG_INFO;
int wpa_debug_show_keys = 0;
int wpa_debug_timestamp = 0;


static int hex2num(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}


static int hex2byte(const char *hex)
{
	int a, b;
	a = hex2num(*hex++);
	if (a < 0)
		return -1;
	b = hex2num(*hex++);
	if (b < 0)
		return -1;
	return (a << 4) | b;
}


int hwaddr_aton(const char *txt, u8 *addr)
{
	int i;

	for (i = 0; i < 6; i++) {
		int a, b;

		a = hex2num(*txt++);
		if (a < 0)
			return -1;
		b = hex2num(*txt++);
		if (b < 0)
			return -1;
		*addr++ = (a << 4) | b;
		if (i < 5 && *txt++ != ':')
			return -1;
	}

	return 0;
}


int hexstr2bin(const char *hex, u8 *buf, size_t len)
{
	size_t i;
	int a;
	const char *ipos = hex;
	u8 *opos = buf;

	for (i = 0; i < len; i++) {
		a = hex2byte(ipos);
		if (a < 0)
			return -1;
		*opos++ = a;
		ipos += 2;
	}
	return 0;
}


void inc_byte_array(u8 *counter, size_t len)
{
	int pos = len - 1;
	while (pos >= 0) {
		counter[pos]++;
		if (counter[pos] != 0)
			break;
		pos--;
	}
}


void wpa_get_ntp_timestamp(u8 *buf)
{
	struct os_time now;
	u32 sec, usec;

	/* 64-bit NTP timestamp (time from 1900-01-01 00:00:00) */
	os_get_time(&now);
	sec = host_to_be32(now.sec + 2208988800U); /* Epoch to 1900 */
	/* Estimate 2^32/10^6 = 4295 - 1/32 - 1/512 */
	usec = now.usec;
	usec = host_to_be32(4295 * usec - (usec >> 5) - (usec >> 9));
	os_memcpy(buf, (u8 *) &sec, 4);
	os_memcpy(buf + 4, (u8 *) &usec, 4);
}

#ifdef ANDROID

#include <android/log.h>

void android_printf(int level, char *format, ...)
{
	if (level >= wpa_debug_level) {
		va_list ap;
		if (level == MSG_ERROR) {
			level = ANDROID_LOG_ERROR;
		} else if (level == MSG_WARNING) {
			level = ANDROID_LOG_WARN;
		} else if (level == MSG_INFO) {
			level = ANDROID_LOG_INFO;
		} else {
			level = ANDROID_LOG_DEBUG;
		}
		va_start(ap, format);
		__android_log_vprint(level, "wpa_supplicant", format, ap);
		va_end(ap);
	}
}

#else /* ANDROID */

#ifndef CONFIG_NO_STDOUT_DEBUG

void wpa_debug_print_timestamp(void)
{
	struct os_time tv;

	if (!wpa_debug_timestamp)
		return;

	os_get_time(&tv);
#ifdef CONFIG_DEBUG_FILE
	if (out_file) {
		fprintf(out_file, "%ld.%06u: ", (long) tv.sec,
			(unsigned int) tv.usec);
	} else
#endif /* CONFIG_DEBUG_FILE */
	printf("%ld.%06u: ", (long) tv.sec, (unsigned int) tv.usec);
}


void wpa_printf(int level, char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	if (level >= wpa_debug_level) {
		wpa_debug_print_timestamp();
#ifdef CONFIG_DEBUG_FILE
		if (out_file) {
			vfprintf(out_file, fmt, ap);
			fprintf(out_file, "\n");
		} else {
#endif /* CONFIG_DEBUG_FILE */
		vprintf(fmt, ap);
		printf("\n");
#ifdef CONFIG_DEBUG_FILE
		}
#endif /* CONFIG_DEBUG_FILE */
	}
	va_end(ap);
}


static void _wpa_hexdump(int level, const char *title, const u8 *buf,
			 size_t len, int show)
{
	size_t i;
	if (level < wpa_debug_level)
		return;
	wpa_debug_print_timestamp();
#ifdef CONFIG_DEBUG_FILE
	if (out_file) {
		fprintf(out_file, "%s - hexdump(len=%lu):",
			title, (unsigned long) len);
		if (buf == NULL) {
			fprintf(out_file, " [NULL]");
		} else if (show) {
			for (i = 0; i < len; i++)
				fprintf(out_file, " %02x", buf[i]);
		} else {
			fprintf(out_file, " [REMOVED]");
		}
		fprintf(out_file, "\n");
	} else {
#endif /* CONFIG_DEBUG_FILE */
	printf("%s - hexdump(len=%lu):", title, (unsigned long) len);
	if (buf == NULL) {
		printf(" [NULL]");
	} else if (show) {
		for (i = 0; i < len; i++)
			printf(" %02x", buf[i]);
	} else {
		printf(" [REMOVED]");
	}
	printf("\n");
#ifdef CONFIG_DEBUG_FILE
	}
#endif /* CONFIG_DEBUG_FILE */
}

void wpa_hexdump(int level, const char *title, const u8 *buf, size_t len)
{
	_wpa_hexdump(level, title, buf, len, 1);
}


void wpa_hexdump_key(int level, const char *title, const u8 *buf, size_t len)
{
	_wpa_hexdump(level, title, buf, len, wpa_debug_show_keys);
}


static void _wpa_hexdump_ascii(int level, const char *title, const u8 *buf,
			       size_t len, int show)
{
	size_t i, llen;
	const u8 *pos = buf;
	const size_t line_len = 16;

	if (level < wpa_debug_level)
		return;
	wpa_debug_print_timestamp();
#ifdef CONFIG_DEBUG_FILE
	if (out_file) {
		if (!show) {
			fprintf(out_file,
				"%s - hexdump_ascii(len=%lu): [REMOVED]\n",
				title, (unsigned long) len);
			return;
		}
		if (buf == NULL) {
			fprintf(out_file,
				"%s - hexdump_ascii(len=%lu): [NULL]\n",
				title, (unsigned long) len);
			return;
		}
		fprintf(out_file, "%s - hexdump_ascii(len=%lu):\n",
			title, (unsigned long) len);
		while (len) {
			llen = len > line_len ? line_len : len;
			fprintf(out_file, "    ");
			for (i = 0; i < llen; i++)
				fprintf(out_file, " %02x", pos[i]);
			for (i = llen; i < line_len; i++)
				fprintf(out_file, "   ");
			fprintf(out_file, "   ");
			for (i = 0; i < llen; i++) {
				if (isprint(pos[i]))
					fprintf(out_file, "%c", pos[i]);
				else
					fprintf(out_file, "_");
			}
			for (i = llen; i < line_len; i++)
				fprintf(out_file, " ");
			fprintf(out_file, "\n");
			pos += llen;
			len -= llen;
		}
	} else {
#endif /* CONFIG_DEBUG_FILE */
	if (!show) {
		printf("%s - hexdump_ascii(len=%lu): [REMOVED]\n",
		       title, (unsigned long) len);
		return;
	}
	if (buf == NULL) {
		printf("%s - hexdump_ascii(len=%lu): [NULL]\n",
		       title, (unsigned long) len);
		return;
	}
	printf("%s - hexdump_ascii(len=%lu):\n", title, (unsigned long) len);
	while (len) {
		llen = len > line_len ? line_len : len;
		printf("    ");
		for (i = 0; i < llen; i++)
			printf(" %02x", pos[i]);
		for (i = llen; i < line_len; i++)
			printf("   ");
		printf("   ");
		for (i = 0; i < llen; i++) {
			if (isprint(pos[i]))
				printf("%c", pos[i]);
			else
				printf("_");
		}
		for (i = llen; i < line_len; i++)
			printf(" ");
		printf("\n");
		pos += llen;
		len -= llen;
	}
#ifdef CONFIG_DEBUG_FILE
	}
#endif /* CONFIG_DEBUG_FILE */
}


void wpa_hexdump_ascii(int level, const char *title, const u8 *buf, size_t len)
{
	_wpa_hexdump_ascii(level, title, buf, len, 1);
}


void wpa_hexdump_ascii_key(int level, const char *title, const u8 *buf,
			   size_t len)
{
	_wpa_hexdump_ascii(level, title, buf, len, wpa_debug_show_keys);
}


int wpa_debug_open_file(const char *path)
{
#ifdef CONFIG_DEBUG_FILE
	if (!path)
		return 0;
	out_file = fopen(path, "a");
	if (out_file == NULL) {
		wpa_printf(MSG_ERROR, "wpa_debug_open_file: Failed to open "
			   "output file, using standard output");
		return -1;
	}
#ifndef _WIN32
	setvbuf(out_file, NULL, _IOLBF, 0);
#endif /* _WIN32 */
#endif /* CONFIG_DEBUG_FILE */
	return 0;
}


void wpa_debug_close_file(void)
{
#ifdef CONFIG_DEBUG_FILE
	if (!out_file)
		return;
	fclose(out_file);
	out_file = NULL;
#endif /* CONFIG_DEBUG_FILE */
}

#endif /* CONFIG_NO_STDOUT_DEBUG */

#endif /* ANDROID */

#ifndef CONFIG_NO_WPA_MSG
static wpa_msg_cb_func wpa_msg_cb = NULL;

void wpa_msg_register_cb(wpa_msg_cb_func func)
{
	wpa_msg_cb = func;
}


void wpa_msg(void *ctx, int level, char *fmt, ...)
{
	va_list ap;
	char *buf;
	const int buflen = 2048;
	int len;

	buf = os_malloc(buflen);
	if (buf == NULL) {
		wpa_printf(MSG_ERROR, "wpa_msg: Failed to allocate message "
			   "buffer");
		return;
	}
	va_start(ap, fmt);
	len = vsnprintf(buf, buflen, fmt, ap);
	va_end(ap);
	wpa_printf(level, "%s", buf);
	if (wpa_msg_cb)
		wpa_msg_cb(ctx, level, buf, len);
	os_free(buf);
}
#endif /* CONFIG_NO_WPA_MSG */


static inline int _wpa_snprintf_hex(char *buf, size_t buf_size, const u8 *data,
				    size_t len, int uppercase)
{
	size_t i;
	char *pos = buf, *end = buf + buf_size;
	int ret;
	if (buf_size == 0)
		return 0;
	for (i = 0; i < len; i++) {
		ret = os_snprintf(pos, end - pos, uppercase ? "%02X" : "%02x",
				  data[i]);
		if (ret < 0 || ret >= end - pos) {
			end[-1] = '\0';
			return pos - buf;
		}
		pos += ret;
	}
	end[-1] = '\0';
	return pos - buf;
}

int wpa_snprintf_hex(char *buf, size_t buf_size, const u8 *data, size_t len)
{
	return _wpa_snprintf_hex(buf, buf_size, data, len, 0);
}


int wpa_snprintf_hex_uppercase(char *buf, size_t buf_size, const u8 *data,
			       size_t len)
{
	return _wpa_snprintf_hex(buf, buf_size, data, len, 1);
}


#ifdef CONFIG_ANSI_C_EXTRA

#ifdef _WIN32_WCE
void perror(const char *s)
{
	wpa_printf(MSG_ERROR, "%s: GetLastError: %d",
		   s, (int) GetLastError());
}
#endif /* _WIN32_WCE */


int optind = 1;
int optopt;
char *optarg;

int getopt(int argc, char *const argv[], const char *optstring)
{
	static int optchr = 1;
	char *cp;

	if (optchr == 1) {
		if (optind >= argc) {
			/* all arguments processed */
			return EOF;
		}

		if (argv[optind][0] != '-' || argv[optind][1] == '\0') {
			/* no option characters */
			return EOF;
		}
	}

	if (os_strcmp(argv[optind], "--") == 0) {
		/* no more options */
		optind++;
		return EOF;
	}

	optopt = argv[optind][optchr];
	cp = os_strchr(optstring, optopt);
	if (cp == NULL || optopt == ':') {
		if (argv[optind][++optchr] == '\0') {
			optchr = 1;
			optind++;
		}
		return '?';
	}

	if (cp[1] == ':') {
		/* Argument required */
		optchr = 1;
		if (argv[optind][optchr + 1]) {
			/* No space between option and argument */
			optarg = &argv[optind++][optchr + 1];
		} else if (++optind >= argc) {
			/* option requires an argument */
			return '?';
		} else {
			/* Argument in the next argv */
			optarg = argv[optind++];
		}
	} else {
		/* No argument */
		if (argv[optind][++optchr] == '\0') {
			optchr = 1;
			optind++;
		}
		optarg = NULL;
	}
	return *cp;
}
#endif /* CONFIG_ANSI_C_EXTRA */


#ifdef CONFIG_NATIVE_WINDOWS
void wpa_unicode2ascii_inplace(TCHAR *str)
{
#ifdef UNICODE
	char *dst = (char *) str;
	while (*str)
		*dst++ = (char) *str++;
	*dst = '\0';
#endif /* UNICODE */
}


TCHAR * wpa_strdup_tchar(const char *str)
{
#ifdef UNICODE
	TCHAR *buf;
	buf = os_malloc((strlen(str) + 1) * sizeof(TCHAR));
	if (buf == NULL)
		return NULL;
	wsprintf(buf, L"%S", str);
	return buf;
#else /* UNICODE */
	return os_strdup(str);
#endif /* UNICODE */
}
#endif /* CONFIG_NATIVE_WINDOWS */


const char * wpa_ssid_txt(u8 *ssid, size_t ssid_len)
{
	static char ssid_txt[33];
	char *pos;

	if (ssid_len > 32)
		ssid_len = 32;
	os_memcpy(ssid_txt, ssid, ssid_len);
	ssid_txt[ssid_len] = '\0';
	for (pos = ssid_txt; *pos != '\0'; pos++) {
#ifndef WPA_UNICODE_SSID
		/* Don't do this, since it prevents us from using APs with non-ASCII SSIDs */
		if ((u8) *pos < 32 || (u8) *pos >= 127)
			*pos = '_';
#endif
	}
	return ssid_txt;
}

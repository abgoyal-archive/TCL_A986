
void info(const char *format, ...) __attribute__((format(printf, 1, 2)));
void error(const char *format, ...) __attribute__((format(printf, 1, 2)));

void btd_debug(const char *format, ...) __attribute__((format(printf, 1, 2)));

void __btd_log_init(const char *debug, int detach);
void __btd_log_cleanup(void);
void __btd_toggle_debug();

struct btd_debug_desc {
	const char *name;
	const char *file;
#define BTD_DEBUG_FLAG_DEFAULT (0)
#define BTD_DEBUG_FLAG_PRINT   (1 << 0)
	unsigned int flags;
} __attribute__((aligned(8)));

#define DBG(fmt, arg...) do { \
	static struct btd_debug_desc __btd_debug_desc \
	__attribute__((used, section("__debug"), aligned(8))) = { \
		.file = __FILE__, .flags = BTD_DEBUG_FLAG_DEFAULT, \
	}; \
	if (__btd_debug_desc.flags & BTD_DEBUG_FLAG_PRINT) \
		btd_debug("%s:%s() " fmt,  __FILE__, __FUNCTION__ , ## arg); \
} while (0)



#ifndef _LINUX_MARKER_H
#define _LINUX_MARKER_H


#include <stdarg.h>
#include <linux/types.h>
#include <linux/immediate.h>

struct module;
struct marker;
struct marker_probe_array;

typedef void marker_probe_func(const struct marker *mdata,
		void *probe_private, void *call_private,
		const char *fmt, va_list *args);

struct marker_probe_closure {
	marker_probe_func *func;	/* Callback */
	void *probe_private;		/* Private probe data */
};

struct marker {
	const char *channel;	/* Name of channel where to send data */
	const char *name;	/* Marker name */
	const char *format;	/* Marker format string, describing the
				 * variable argument list.
				 */
	DEFINE_IMV(char, state);/* Immediate value state. */
	char ptype;		/* probe type : 0 : single, 1 : multi */
				/* Probe wrapper */
	u16 channel_id;		/* Numeric channel identifier, dynamic */
	u16 event_id;		/* Numeric event identifier, dynamic */
	void (*call)(const struct marker *mdata, void *call_private, ...);
	struct marker_probe_closure single;
	struct marker_probe_array *multi;
	const char *tp_name;	/* Optional tracepoint name */
	void *tp_cb;		/* Optional tracepoint callback */
} __attribute__((aligned(128)));	/*
					 * Aligned on 128 bytes because it is
					 * globally visible and gcc happily
					 * align these on the structure size.
					 * Keep in sync with vmlinux.lds.h.
					 */

#ifdef CONFIG_MARKERS

#define _DEFINE_MARKER(channel, name, tp_name_str, tp_cb, format)	\
		static const char __mstrtab_##channel##_##name[]	\
		__attribute__((section("__markers_strings")))		\
		= #channel "\0" #name "\0" format;			\
		static struct marker __mark_##channel##_##name		\
		__attribute__((section("__markers"), aligned(8))) =	\
		{ __mstrtab_##channel##_##name,				\
		  &__mstrtab_##channel##_##name[sizeof(#channel)],	\
		  &__mstrtab_##channel##_##name[sizeof(#channel) +	\
						sizeof(#name)],		\
		  0, 0, 0, 0, marker_probe_cb,				\
		  { __mark_empty_function, NULL},			\
		  NULL, tp_name_str, tp_cb }

#define DEFINE_MARKER(channel, name, format)				\
		_DEFINE_MARKER(channel, name, NULL, NULL, format)

#define DEFINE_MARKER_TP(channel, name, tp_name, tp_cb, format)		\
		_DEFINE_MARKER(channel, name, #tp_name, tp_cb, format)

#define __trace_mark(generic, channel, name, call_private, format, args...) \
	do {								\
		DEFINE_MARKER(channel, name, format);			\
		__mark_check_format(format, ## args);			\
		if (!generic) {						\
			if (unlikely(imv_read(				\
					__mark_##channel##_##name.state))) \
				(*__mark_##channel##_##name.call)	\
					(&__mark_##channel##_##name,	\
					call_private, ## args);		\
		} else {						\
			if (unlikely(_imv_read(				\
					__mark_##channel##_##name.state))) \
				(*__mark_##channel##_##name.call)	\
					(&__mark_##channel##_##name,	\
					call_private, ## args);		\
		}							\
	} while (0)

#define __trace_mark_tp(channel, name, call_private, tp_name, tp_cb,	\
			format, args...)				\
	do {								\
		void __check_tp_type(void)				\
		{							\
			register_trace_##tp_name(tp_cb, NULL);		\
		}							\
		DEFINE_MARKER_TP(channel, name, tp_name, tp_cb, format);\
		__mark_check_format(format, ## args);			\
		(*__mark_##channel##_##name.call)(&__mark_##channel##_##name, \
			call_private, ## args);				\
	} while (0)

extern void marker_update_probe_range(struct marker *begin,
	struct marker *end);

#define GET_MARKER(channel, name)	(__mark_##channel##_##name)

#else /* !CONFIG_MARKERS */
#define DEFINE_MARKER(channel, name, tp_name, tp_cb, format)
#define __trace_mark(generic, channel, name, call_private, format, args...) \
		__mark_check_format(format, ## args)
#define __trace_mark_tp(channel, name, call_private, tp_name, tp_cb,	\
		format, args...)					\
	do {								\
		void __check_tp_type(void)				\
		{							\
			register_trace_##tp_name(tp_cb, NULL);		\
		}							\
		__mark_check_format(format, ## args);			\
	} while (0)
static inline void marker_update_probe_range(struct marker *begin,
	struct marker *end)
{ }
#define GET_MARKER(channel, name)
#endif /* CONFIG_MARKERS */

#define trace_mark(channel, name, format, args...) \
	__trace_mark(0, channel, name, NULL, format, ## args)

#define _trace_mark(channel, name, format, args...) \
	__trace_mark(1, channel, name, NULL, format, ## args)

#define trace_mark_tp(channel, name, tp_name, tp_cb, format, args...)	\
	__trace_mark_tp(channel, name, NULL, tp_name, tp_cb, format, ## args)

#define MARK_NOARGS " "

extern void lock_markers(void);
extern void unlock_markers(void);

extern void markers_compact_event_ids(void);

/* To be used for string format validity checking with gcc */
static inline void __printf(1, 2) ___mark_check_format(const char *fmt, ...)
{
}

#define __mark_check_format(format, args...)				\
	do {								\
		if (0)							\
			___mark_check_format(format, ## args);		\
	} while (0)

extern marker_probe_func __mark_empty_function;

extern void marker_probe_cb(const struct marker *mdata,
	void *call_private, ...);

extern int marker_probe_register(const char *channel, const char *name,
	const char *format, marker_probe_func *probe, void *probe_private);

extern int marker_probe_unregister(const char *channel, const char *name,
	marker_probe_func *probe, void *probe_private);
extern int marker_probe_unregister_private_data(marker_probe_func *probe,
	void *probe_private);

extern void *marker_get_private_data(const char *channel, const char *name,
	marker_probe_func *probe, int num);

const char *marker_get_name_from_id(u16 channel_id, u16 event_id);
const char *marker_get_fmt_from_id(u16 channel_id, u16 event_id);

#define marker_synchronize_unregister() synchronize_sched()

struct marker_iter {
	struct module *module;
	struct marker *marker;
};

extern void marker_iter_start(struct marker_iter *iter);
extern void marker_iter_next(struct marker_iter *iter);
extern void marker_iter_stop(struct marker_iter *iter);
extern void marker_iter_reset(struct marker_iter *iter);
extern int marker_get_iter_range(struct marker **marker, struct marker *begin,
	struct marker *end);
extern int _is_marker_enabled(const char *channel, const char *name);
extern int is_marker_enabled(const char *channel, const char *name);
extern int is_marker_present(const char *channel, const char *name);
extern void marker_update_probes(void);

#endif

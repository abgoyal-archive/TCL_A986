
#ifndef _LINUX_TRACEPOINT_H
#define _LINUX_TRACEPOINT_H


#include <linux/immediate.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/rcupdate.h>

struct module;
struct tracepoint;

struct tracepoint_func {
	void *func;
	void *data;
};

struct tracepoint {
	const char *name;		/* Tracepoint name */
	DEFINE_IMV(char, state);	/* State. */
	void (*regfunc)(void);
	void (*unregfunc)(void);
	struct tracepoint_func *funcs;
} __attribute__((aligned(32)));		/*
					 * Aligned on 32 bytes because it is
					 * globally visible and gcc happily
					 * align these on the structure size.
					 * Keep in sync with vmlinux.lds.h.
					 */

extern int tracepoint_probe_register(const char *name, void *probe, void *data);

extern int
tracepoint_probe_unregister(const char *name, void *probe, void *data);

extern int tracepoint_probe_register_noupdate(const char *name, void *probe,
					      void *data);
extern int tracepoint_probe_unregister_noupdate(const char *name, void *probe,
						void *data);
extern void tracepoint_probe_update_all(void);

struct tracepoint_iter {
	struct module *module;
	struct tracepoint *tracepoint;
};

extern void tracepoint_iter_start(struct tracepoint_iter *iter);
extern void tracepoint_iter_next(struct tracepoint_iter *iter);
extern void tracepoint_iter_stop(struct tracepoint_iter *iter);
extern void tracepoint_iter_reset(struct tracepoint_iter *iter);
extern int tracepoint_get_iter_range(struct tracepoint **tracepoint,
	struct tracepoint *begin, struct tracepoint *end);

static inline void tracepoint_synchronize_unregister(void)
{
	synchronize_sched();
}

#define PARAMS(args...) args

#ifdef CONFIG_TRACEPOINTS
extern void tracepoint_update_probe_range(struct tracepoint *begin,
	struct tracepoint *end);
#else
static inline void tracepoint_update_probe_range(struct tracepoint *begin,
	struct tracepoint *end)
{ }
#endif /* CONFIG_TRACEPOINTS */

#endif /* _LINUX_TRACEPOINT_H */


#ifndef DECLARE_TRACE

#define TP_PROTO(args...)	args
#define TP_ARGS(args...)	args

#define DECLARE_TRACE_NOP(name, proto, args, data_proto, data_args)	\
	static inline void trace_##name(proto)				\
	{ }								\
	static inline void _trace_##name(proto)				\
	{ }								\
	static inline int register_trace_##name(void (*probe)(data_proto), \
						void *data)		\
	{								\
		return -ENOSYS;						\
	}								\
	static inline int unregister_trace_##name(void (*probe)(data_proto), \
						  void *data)		\
	{								\
		return -ENOSYS;						\
	}								\
	static inline void check_trace_callback_type_##name(void (*cb)(data_proto)) \
	{								\
	}

#define DEFINE_TRACE_FN_NOP(name, reg, unreg)
#define DEFINE_TRACE_NOP(name)
#define EXPORT_TRACEPOINT_SYMBOL_GPL_NOP(name)
#define EXPORT_TRACEPOINT_SYMBOL_NOP(name)

#ifdef CONFIG_TRACEPOINTS

#define __DO_TRACE(tp, proto, args)					\
	do {								\
		struct tracepoint_func *it_func_ptr;			\
		void *it_func;						\
		void *__data;						\
									\
		rcu_read_lock_sched_notrace();				\
		it_func_ptr = rcu_dereference_sched((tp)->funcs);	\
		if (it_func_ptr) {					\
			do {						\
				it_func = (it_func_ptr)->func;		\
				__data = (it_func_ptr)->data;		\
				((void(*)(proto))(it_func))(args);	\
			} while ((++it_func_ptr)->func);		\
		}							\
		rcu_read_unlock_sched_notrace();			\
	} while (0)

#define __CHECK_TRACE(name, generic, proto, args)			\
	do {								\
		if (!generic) {						\
			if (unlikely(imv_read(__tracepoint_##name.state))) \
				__DO_TRACE(&__tracepoint_##name,	\
					TP_PROTO(proto), TP_ARGS(args));\
		} else {						\
			if (unlikely(_imv_read(__tracepoint_##name.state))) \
				__DO_TRACE(&__tracepoint_##name,	\
					TP_PROTO(proto), TP_ARGS(args));\
		}							\
	} while (0)

#define __DECLARE_TRACE(name, proto, args, data_proto, data_args)	\
	extern struct tracepoint __tracepoint_##name;			\
	static inline void trace_##name(proto)				\
	{								\
		__CHECK_TRACE(name, 0, TP_PROTO(data_proto),		\
			      TP_ARGS(data_args));			\
	}								\
	static inline void _trace_##name(proto)				\
	{								\
		__CHECK_TRACE(name, 1, TP_PROTO(data_proto),		\
			      TP_ARGS(data_args));			\
	}								\
	static inline int						\
	register_trace_##name(void (*probe)(data_proto), void *data)	\
	{								\
		return tracepoint_probe_register(#name, (void *)probe,	\
						 data);			\
	}								\
	static inline int						\
	unregister_trace_##name(void (*probe)(data_proto), void *data)	\
	{								\
		return tracepoint_probe_unregister(#name, (void *)probe, \
						   data);		\
	}								\
	static inline void						\
	check_trace_callback_type_##name(void (*cb)(data_proto))	\
	{								\
	}

#define DEFINE_TRACE_FN(name, reg, unreg)				\
	static const char __tpstrtab_##name[]				\
	__attribute__((section("__tracepoints_strings"))) = #name;	\
	struct tracepoint __tracepoint_##name				\
	__attribute__((section("__tracepoints"), aligned(32))) =	\
		{ __tpstrtab_##name, 0, reg, unreg, NULL }

#define DEFINE_TRACE(name)						\
	DEFINE_TRACE_FN(name, NULL, NULL);

#define EXPORT_TRACEPOINT_SYMBOL_GPL(name)				\
	EXPORT_SYMBOL_GPL(__tracepoint_##name)
#define EXPORT_TRACEPOINT_SYMBOL(name)					\
	EXPORT_SYMBOL(__tracepoint_##name)

#else /* !CONFIG_TRACEPOINTS */

#define DEFINE_TRACE_FN			DEFINE_TRACE_FN_NOP
#define DEFINE_TRACE			DECLARE_TRACE_NOP
#define EXPORT_TRACEPOINT_SYMBOL_GPL	EXPORT_TRACEPOINT_SYMBOL_GPL_NOP
#define EXPORT_TRACEPOINT_SYMBOL	EXPORT_TRACEPOINT_SYMBOL_NOP

#endif /* CONFIG_TRACEPOINTS */

#define DECLARE_TRACE_NOARGS(name)					\
		__DECLARE_TRACE(name, void, , void *__data, __data)

#define DECLARE_TRACE(name, proto, args)				\
		__DECLARE_TRACE(name, PARAMS(proto), PARAMS(args),	\
				PARAMS(void *__data, proto),		\
				PARAMS(__data, args))

#endif /* DECLARE_TRACE */

#ifndef TRACE_EVENT

#define DECLARE_EVENT_CLASS(name, proto, args, tstruct, assign, print)
#define DEFINE_EVENT(template, name, proto, args)		\
	DECLARE_TRACE(name, PARAMS(proto), PARAMS(args))
#define DEFINE_EVENT_PRINT(template, name, proto, args, print)	\
	DECLARE_TRACE(name, PARAMS(proto), PARAMS(args))

#define TRACE_EVENT(name, proto, args, struct, assign, print)	\
	DECLARE_TRACE(name, PARAMS(proto), PARAMS(args))
#define TRACE_EVENT_FN(name, proto, args, struct,		\
		assign, print, reg, unreg)			\
	DECLARE_TRACE(name, PARAMS(proto), PARAMS(args))

#endif /* ifdef TRACE_EVENT (see note above) */

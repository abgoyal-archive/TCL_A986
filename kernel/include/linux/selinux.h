
#ifndef _LINUX_SELINUX_H
#define _LINUX_SELINUX_H

struct selinux_audit_rule;
struct audit_context;
struct kern_ipc_perm;

#ifdef CONFIG_SECURITY_SELINUX

int selinux_string_to_sid(char *str, u32 *sid);

int selinux_secmark_relabel_packet_permission(u32 sid);

void selinux_secmark_refcount_inc(void);

void selinux_secmark_refcount_dec(void);

bool selinux_is_enabled(void);
#else

static inline int selinux_string_to_sid(const char *str, u32 *sid)
{
       *sid = 0;
       return 0;
}

static inline int selinux_secmark_relabel_packet_permission(u32 sid)
{
	return 0;
}

static inline void selinux_secmark_refcount_inc(void)
{
	return;
}

static inline void selinux_secmark_refcount_dec(void)
{
	return;
}

static inline bool selinux_is_enabled(void)
{
	return false;
}
#endif	/* CONFIG_SECURITY_SELINUX */

#endif /* _LINUX_SELINUX_H */

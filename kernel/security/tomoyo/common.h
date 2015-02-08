

#ifndef _SECURITY_TOMOYO_COMMON_H
#define _SECURITY_TOMOYO_COMMON_H

#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/file.h>
#include <linux/kmod.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/list.h>
#include <linux/cred.h>
struct linux_binprm;

/********** Constants definitions. **********/

#define TOMOYO_HASH_BITS  8
#define TOMOYO_MAX_HASH (1u<<TOMOYO_HASH_BITS)

#define TOMOYO_MAX_PATHNAME_LEN 4000

/* Profile number is an integer between 0 and 255. */
#define TOMOYO_MAX_PROFILES 256

/* Keywords for ACLs. */
#define TOMOYO_KEYWORD_ALIAS                     "alias "
#define TOMOYO_KEYWORD_ALLOW_READ                "allow_read "
#define TOMOYO_KEYWORD_DELETE                    "delete "
#define TOMOYO_KEYWORD_DENY_REWRITE              "deny_rewrite "
#define TOMOYO_KEYWORD_FILE_PATTERN              "file_pattern "
#define TOMOYO_KEYWORD_INITIALIZE_DOMAIN         "initialize_domain "
#define TOMOYO_KEYWORD_KEEP_DOMAIN               "keep_domain "
#define TOMOYO_KEYWORD_NO_INITIALIZE_DOMAIN      "no_initialize_domain "
#define TOMOYO_KEYWORD_NO_KEEP_DOMAIN            "no_keep_domain "
#define TOMOYO_KEYWORD_PATH_GROUP                "path_group "
#define TOMOYO_KEYWORD_SELECT                    "select "
#define TOMOYO_KEYWORD_USE_PROFILE               "use_profile "
#define TOMOYO_KEYWORD_IGNORE_GLOBAL_ALLOW_READ  "ignore_global_allow_read"
/* A domain definition starts with <kernel>. */
#define TOMOYO_ROOT_NAME                         "<kernel>"
#define TOMOYO_ROOT_NAME_LEN                     (sizeof(TOMOYO_ROOT_NAME) - 1)

/* Index numbers for Access Controls. */
enum tomoyo_mac_index {
	TOMOYO_MAC_FOR_FILE,  /* domain_policy.conf */
	TOMOYO_MAX_ACCEPT_ENTRY,
	TOMOYO_VERBOSE,
	TOMOYO_MAX_CONTROL_INDEX
};

/* Index numbers for Access Controls. */
enum tomoyo_acl_entry_type_index {
	TOMOYO_TYPE_PATH_ACL,
	TOMOYO_TYPE_PATH2_ACL,
};

/* Index numbers for File Controls. */


enum tomoyo_path_acl_index {
	TOMOYO_TYPE_READ_WRITE,
	TOMOYO_TYPE_EXECUTE,
	TOMOYO_TYPE_READ,
	TOMOYO_TYPE_WRITE,
	TOMOYO_TYPE_CREATE,
	TOMOYO_TYPE_UNLINK,
	TOMOYO_TYPE_MKDIR,
	TOMOYO_TYPE_RMDIR,
	TOMOYO_TYPE_MKFIFO,
	TOMOYO_TYPE_MKSOCK,
	TOMOYO_TYPE_MKBLOCK,
	TOMOYO_TYPE_MKCHAR,
	TOMOYO_TYPE_TRUNCATE,
	TOMOYO_TYPE_SYMLINK,
	TOMOYO_TYPE_REWRITE,
	TOMOYO_TYPE_IOCTL,
	TOMOYO_TYPE_CHMOD,
	TOMOYO_TYPE_CHOWN,
	TOMOYO_TYPE_CHGRP,
	TOMOYO_TYPE_CHROOT,
	TOMOYO_TYPE_MOUNT,
	TOMOYO_TYPE_UMOUNT,
	TOMOYO_MAX_PATH_OPERATION
};

enum tomoyo_path2_acl_index {
	TOMOYO_TYPE_LINK,
	TOMOYO_TYPE_RENAME,
	TOMOYO_TYPE_PIVOT_ROOT,
	TOMOYO_MAX_PATH2_OPERATION
};

enum tomoyo_securityfs_interface_index {
	TOMOYO_DOMAINPOLICY,
	TOMOYO_EXCEPTIONPOLICY,
	TOMOYO_DOMAIN_STATUS,
	TOMOYO_PROCESS_STATUS,
	TOMOYO_MEMINFO,
	TOMOYO_SELFDOMAIN,
	TOMOYO_VERSION,
	TOMOYO_PROFILE,
	TOMOYO_MANAGER
};

/********** Structure definitions. **********/

struct tomoyo_page_buffer {
	char buffer[4096];
};

struct tomoyo_path_info {
	const char *name;
	u32 hash;          /* = full_name_hash(name, strlen(name)) */
	u16 const_len;     /* = tomoyo_const_part_length(name)     */
	bool is_dir;       /* = tomoyo_strendswith(name, "/")      */
	bool is_patterned; /* = tomoyo_path_contains_pattern(name) */
};

struct tomoyo_name_entry {
	struct list_head list;
	atomic_t users;
	struct tomoyo_path_info entry;
};

struct tomoyo_path_info_with_data {
	/* Keep "head" first, for this pointer is passed to kfree(). */
	struct tomoyo_path_info head;
	char barrier1[16]; /* Safeguard for overrun. */
	char body[TOMOYO_MAX_PATHNAME_LEN];
	char barrier2[16]; /* Safeguard for overrun. */
};

struct tomoyo_name_union {
	const struct tomoyo_path_info *filename;
	struct tomoyo_path_group *group;
	u8 is_group;
};

/* Structure for "path_group" directive. */
struct tomoyo_path_group {
	struct list_head list;
	const struct tomoyo_path_info *group_name;
	struct list_head member_list;
	atomic_t users;
};

/* Structure for "path_group" directive. */
struct tomoyo_path_group_member {
	struct list_head list;
	bool is_deleted;
	const struct tomoyo_path_info *member_name;
};

struct tomoyo_acl_info {
	struct list_head list;
	u8 type;
} __packed;

struct tomoyo_domain_info {
	struct list_head list;
	struct list_head acl_info_list;
	/* Name of this domain. Never NULL.          */
	const struct tomoyo_path_info *domainname;
	u8 profile;        /* Profile number to use. */
	bool is_deleted;   /* Delete flag.           */
	bool quota_warned; /* Quota warnning flag.   */
	bool ignore_global_allow_read; /* Ignore "allow_read" flag. */
	bool transition_failed; /* Domain transition failed flag. */
	atomic_t users; /* Number of referring credentials. */
};

struct tomoyo_path_acl {
	struct tomoyo_acl_info head; /* type = TOMOYO_TYPE_PATH_ACL */
	u8 perm_high;
	u16 perm;
	struct tomoyo_name_union name;
};

struct tomoyo_path2_acl {
	struct tomoyo_acl_info head; /* type = TOMOYO_TYPE_PATH2_ACL */
	u8 perm;
	struct tomoyo_name_union name1;
	struct tomoyo_name_union name2;
};

struct tomoyo_io_buffer {
	int (*read) (struct tomoyo_io_buffer *);
	int (*write) (struct tomoyo_io_buffer *);
	/* Exclusive lock for this structure.   */
	struct mutex io_sem;
	/* Index returned by tomoyo_read_lock(). */
	int reader_idx;
	/* The position currently reading from. */
	struct list_head *read_var1;
	/* Extra variables for reading.         */
	struct list_head *read_var2;
	/* The position currently writing to.   */
	struct tomoyo_domain_info *write_var1;
	/* The step for reading.                */
	int read_step;
	/* Buffer for reading.                  */
	char *read_buf;
	/* EOF flag for reading.                */
	bool read_eof;
	/* Read domain ACL of specified PID?    */
	bool read_single_domain;
	/* Extra variable for reading.          */
	u8 read_bit;
	/* Bytes available for reading.         */
	int read_avail;
	/* Size of read buffer.                 */
	int readbuf_size;
	/* Buffer for writing.                  */
	char *write_buf;
	/* Bytes available for writing.         */
	int write_avail;
	/* Size of write buffer.                */
	int writebuf_size;
};

struct tomoyo_globally_readable_file_entry {
	struct list_head list;
	const struct tomoyo_path_info *filename;
	bool is_deleted;
};

struct tomoyo_pattern_entry {
	struct list_head list;
	const struct tomoyo_path_info *pattern;
	bool is_deleted;
};

struct tomoyo_no_rewrite_entry {
	struct list_head list;
	const struct tomoyo_path_info *pattern;
	bool is_deleted;
};

struct tomoyo_domain_initializer_entry {
	struct list_head list;
	const struct tomoyo_path_info *domainname;    /* This may be NULL */
	const struct tomoyo_path_info *program;
	bool is_deleted;
	bool is_not;       /* True if this entry is "no_initialize_domain".  */
	/* True if the domainname is tomoyo_get_last_name(). */
	bool is_last_name;
};

struct tomoyo_domain_keeper_entry {
	struct list_head list;
	const struct tomoyo_path_info *domainname;
	const struct tomoyo_path_info *program;       /* This may be NULL */
	bool is_deleted;
	bool is_not;       /* True if this entry is "no_keep_domain".        */
	/* True if the domainname is tomoyo_get_last_name(). */
	bool is_last_name;
};

struct tomoyo_alias_entry {
	struct list_head list;
	const struct tomoyo_path_info *original_name;
	const struct tomoyo_path_info *aliased_name;
	bool is_deleted;
};

struct tomoyo_policy_manager_entry {
	struct list_head list;
	/* A path to program or a domainname. */
	const struct tomoyo_path_info *manager;
	bool is_domain;  /* True if manager is a domainname. */
	bool is_deleted; /* True if this entry is deleted. */
};

/********** Function prototypes. **********/

/* Check whether the given name matches the given name_union. */
bool tomoyo_compare_name_union(const struct tomoyo_path_info *name,
			       const struct tomoyo_name_union *ptr);
/* Check whether the domain has too many ACL entries to hold. */
bool tomoyo_domain_quota_is_ok(struct tomoyo_domain_info * const domain);
/* Transactional sprintf() for policy dump. */
bool tomoyo_io_printf(struct tomoyo_io_buffer *head, const char *fmt, ...)
	__attribute__ ((format(printf, 2, 3)));
/* Check whether the domainname is correct. */
bool tomoyo_is_correct_domain(const unsigned char *domainname);
/* Check whether the token is correct. */
bool tomoyo_is_correct_path(const char *filename, const s8 start_type,
			    const s8 pattern_type, const s8 end_type);
/* Check whether the token can be a domainname. */
bool tomoyo_is_domain_def(const unsigned char *buffer);
bool tomoyo_parse_name_union(const char *filename,
			     struct tomoyo_name_union *ptr);
/* Check whether the given filename matches the given path_group. */
bool tomoyo_path_matches_group(const struct tomoyo_path_info *pathname,
			       const struct tomoyo_path_group *group,
			       const bool may_use_pattern);
/* Check whether the given filename matches the given pattern. */
bool tomoyo_path_matches_pattern(const struct tomoyo_path_info *filename,
				 const struct tomoyo_path_info *pattern);
/* Read "alias" entry in exception policy. */
bool tomoyo_read_alias_policy(struct tomoyo_io_buffer *head);
bool tomoyo_read_domain_initializer_policy(struct tomoyo_io_buffer *head);
/* Read "keep_domain" and "no_keep_domain" entry in exception policy. */
bool tomoyo_read_domain_keeper_policy(struct tomoyo_io_buffer *head);
/* Read "file_pattern" entry in exception policy. */
bool tomoyo_read_file_pattern(struct tomoyo_io_buffer *head);
/* Read "path_group" entry in exception policy. */
bool tomoyo_read_path_group_policy(struct tomoyo_io_buffer *head);
/* Read "allow_read" entry in exception policy. */
bool tomoyo_read_globally_readable_policy(struct tomoyo_io_buffer *head);
/* Read "deny_rewrite" entry in exception policy. */
bool tomoyo_read_no_rewrite_policy(struct tomoyo_io_buffer *head);
/* Tokenize a line. */
bool tomoyo_tokenize(char *buffer, char *w[], size_t size);
/* Write domain policy violation warning message to console? */
bool tomoyo_verbose_mode(const struct tomoyo_domain_info *domain);
/* Convert double path operation to operation name. */
const char *tomoyo_path22keyword(const u8 operation);
/* Get the last component of the given domainname. */
const char *tomoyo_get_last_name(const struct tomoyo_domain_info *domain);
/* Get warning message. */
const char *tomoyo_get_msg(const bool is_enforce);
/* Convert single path operation to operation name. */
const char *tomoyo_path2keyword(const u8 operation);
/* Create "alias" entry in exception policy. */
int tomoyo_write_alias_policy(char *data, const bool is_delete);
int tomoyo_write_domain_initializer_policy(char *data, const bool is_not,
					   const bool is_delete);
/* Create "keep_domain" and "no_keep_domain" entry in exception policy. */
int tomoyo_write_domain_keeper_policy(char *data, const bool is_not,
				      const bool is_delete);
int tomoyo_write_file_policy(char *data, struct tomoyo_domain_info *domain,
			     const bool is_delete);
/* Create "allow_read" entry in exception policy. */
int tomoyo_write_globally_readable_policy(char *data, const bool is_delete);
/* Create "deny_rewrite" entry in exception policy. */
int tomoyo_write_no_rewrite_policy(char *data, const bool is_delete);
/* Create "file_pattern" entry in exception policy. */
int tomoyo_write_pattern_policy(char *data, const bool is_delete);
/* Create "path_group" entry in exception policy. */
int tomoyo_write_path_group_policy(char *data, const bool is_delete);
/* Find a domain by the given name. */
struct tomoyo_domain_info *tomoyo_find_domain(const char *domainname);
/* Find or create a domain by the given name. */
struct tomoyo_domain_info *tomoyo_find_or_assign_new_domain(const char *
							    domainname,
							    const u8 profile);

/* Allocate memory for "struct tomoyo_path_group". */
struct tomoyo_path_group *tomoyo_get_path_group(const char *group_name);

/* Check mode for specified functionality. */
unsigned int tomoyo_check_flags(const struct tomoyo_domain_info *domain,
				const u8 index);
/* Fill in "struct tomoyo_path_info" members. */
void tomoyo_fill_path_info(struct tomoyo_path_info *ptr);
/* Run policy loader when /sbin/init starts. */
void tomoyo_load_policy(const char *filename);

/* Convert binary string to ascii string. */
int tomoyo_encode(char *buffer, int buflen, const char *str);

/* Returns realpath(3) of the given pathname but ignores chroot'ed root. */
int tomoyo_realpath_from_path2(struct path *path, char *newname,
			       int newname_len);

char *tomoyo_realpath(const char *pathname);
char *tomoyo_realpath_nofollow(const char *pathname);
/* Same with tomoyo_realpath() except that the pathname is already solved. */
char *tomoyo_realpath_from_path(struct path *path);

/* Check memory quota. */
bool tomoyo_memory_ok(void *ptr);
void *tomoyo_commit_ok(void *data, const unsigned int size);

const struct tomoyo_path_info *tomoyo_get_name(const char *name);

/* Check for memory usage. */
int tomoyo_read_memory_counter(struct tomoyo_io_buffer *head);

/* Set memory quota. */
int tomoyo_write_memory_quota(struct tomoyo_io_buffer *head);

/* Initialize realpath related code. */
void __init tomoyo_realpath_init(void);
int tomoyo_check_exec_perm(struct tomoyo_domain_info *domain,
			   const struct tomoyo_path_info *filename);
int tomoyo_check_open_permission(struct tomoyo_domain_info *domain,
				 struct path *path, const int flag);
int tomoyo_path_perm(const u8 operation, struct path *path);
int tomoyo_path2_perm(const u8 operation, struct path *path1,
		      struct path *path2);
int tomoyo_check_rewrite_permission(struct file *filp);
int tomoyo_find_next_domain(struct linux_binprm *bprm);

/* Drop refcount on tomoyo_name_union. */
void tomoyo_put_name_union(struct tomoyo_name_union *ptr);

/* Run garbage collector. */
void tomoyo_run_gc(void);

void tomoyo_memory_free(void *ptr);

/********** External variable definitions. **********/

/* Lock for GC. */
extern struct srcu_struct tomoyo_ss;

/* The list for "struct tomoyo_domain_info". */
extern struct list_head tomoyo_domain_list;

extern struct list_head tomoyo_path_group_list;
extern struct list_head tomoyo_domain_initializer_list;
extern struct list_head tomoyo_domain_keeper_list;
extern struct list_head tomoyo_alias_list;
extern struct list_head tomoyo_globally_readable_list;
extern struct list_head tomoyo_pattern_list;
extern struct list_head tomoyo_no_rewrite_list;
extern struct list_head tomoyo_policy_manager_list;
extern struct list_head tomoyo_name_list[TOMOYO_MAX_HASH];

/* Lock for protecting policy. */
extern struct mutex tomoyo_policy_lock;

/* Has /sbin/init started? */
extern bool tomoyo_policy_loaded;

/* The kernel's domain. */
extern struct tomoyo_domain_info tomoyo_kernel_domain;

/********** Inlined functions. **********/

static inline int tomoyo_read_lock(void)
{
	return srcu_read_lock(&tomoyo_ss);
}

static inline void tomoyo_read_unlock(int idx)
{
	srcu_read_unlock(&tomoyo_ss, idx);
}

/* strcmp() for "struct tomoyo_path_info" structure. */
static inline bool tomoyo_pathcmp(const struct tomoyo_path_info *a,
				  const struct tomoyo_path_info *b)
{
	return a->hash != b->hash || strcmp(a->name, b->name);
}

static inline bool tomoyo_is_valid(const unsigned char c)
{
	return c > ' ' && c < 127;
}

static inline bool tomoyo_is_invalid(const unsigned char c)
{
	return c && (c <= ' ' || c >= 127);
}

static inline void tomoyo_put_name(const struct tomoyo_path_info *name)
{
	if (name) {
		struct tomoyo_name_entry *ptr =
			container_of(name, struct tomoyo_name_entry, entry);
		atomic_dec(&ptr->users);
	}
}

static inline void tomoyo_put_path_group(struct tomoyo_path_group *group)
{
	if (group)
		atomic_dec(&group->users);
}

static inline struct tomoyo_domain_info *tomoyo_domain(void)
{
	return current_cred()->security;
}

static inline struct tomoyo_domain_info *tomoyo_real_domain(struct task_struct
							    *task)
{
	return task_cred_xxx(task, security);
}

static inline bool tomoyo_is_same_acl_head(const struct tomoyo_acl_info *p1,
					   const struct tomoyo_acl_info *p2)
{
	return p1->type == p2->type;
}

static inline bool tomoyo_is_same_name_union
(const struct tomoyo_name_union *p1, const struct tomoyo_name_union *p2)
{
	return p1->filename == p2->filename && p1->group == p2->group &&
		p1->is_group == p2->is_group;
}

static inline bool tomoyo_is_same_path_acl(const struct tomoyo_path_acl *p1,
					   const struct tomoyo_path_acl *p2)
{
	return tomoyo_is_same_acl_head(&p1->head, &p2->head) &&
		tomoyo_is_same_name_union(&p1->name, &p2->name);
}

static inline bool tomoyo_is_same_path2_acl(const struct tomoyo_path2_acl *p1,
					    const struct tomoyo_path2_acl *p2)
{
	return tomoyo_is_same_acl_head(&p1->head, &p2->head) &&
		tomoyo_is_same_name_union(&p1->name1, &p2->name1) &&
		tomoyo_is_same_name_union(&p1->name2, &p2->name2);
}

static inline bool tomoyo_is_same_domain_initializer_entry
(const struct tomoyo_domain_initializer_entry *p1,
 const struct tomoyo_domain_initializer_entry *p2)
{
	return p1->is_not == p2->is_not && p1->is_last_name == p2->is_last_name
		&& p1->domainname == p2->domainname
		&& p1->program == p2->program;
}

static inline bool tomoyo_is_same_domain_keeper_entry
(const struct tomoyo_domain_keeper_entry *p1,
 const struct tomoyo_domain_keeper_entry *p2)
{
	return p1->is_not == p2->is_not && p1->is_last_name == p2->is_last_name
		&& p1->domainname == p2->domainname
		&& p1->program == p2->program;
}

static inline bool tomoyo_is_same_alias_entry
(const struct tomoyo_alias_entry *p1, const struct tomoyo_alias_entry *p2)
{
	return p1->original_name == p2->original_name &&
		p1->aliased_name == p2->aliased_name;
}

#define list_for_each_cookie(pos, cookie, head)				\
	for (({ if (!cookie)						\
				     cookie = head; }),			\
		     pos = rcu_dereference((cookie)->next);		\
	     prefetch(pos->next), pos != (head) || ((cookie) = NULL);	\
	     (cookie) = pos, pos = rcu_dereference(pos->next))

#endif /* !defined(_SECURITY_TOMOYO_COMMON_H) */

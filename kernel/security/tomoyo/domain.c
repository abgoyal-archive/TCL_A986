

#include "common.h"
#include <linux/binfmts.h>
#include <linux/slab.h>

/* Variables definitions.*/

/* The initial domain. */
struct tomoyo_domain_info tomoyo_kernel_domain;

LIST_HEAD(tomoyo_domain_list);

const char *tomoyo_get_last_name(const struct tomoyo_domain_info *domain)
{
	const char *cp0 = domain->domainname->name;
	const char *cp1 = strrchr(cp0, ' ');

	if (cp1)
		return cp1 + 1;
	return cp0;
}

LIST_HEAD(tomoyo_domain_initializer_list);

static int tomoyo_update_domain_initializer_entry(const char *domainname,
						  const char *program,
						  const bool is_not,
						  const bool is_delete)
{
	struct tomoyo_domain_initializer_entry *ptr;
	struct tomoyo_domain_initializer_entry e = { .is_not = is_not };
	int error = is_delete ? -ENOENT : -ENOMEM;

	if (!tomoyo_is_correct_path(program, 1, -1, -1))
		return -EINVAL; /* No patterns allowed. */
	if (domainname) {
		if (!tomoyo_is_domain_def(domainname) &&
		    tomoyo_is_correct_path(domainname, 1, -1, -1))
			e.is_last_name = true;
		else if (!tomoyo_is_correct_domain(domainname))
			return -EINVAL;
		e.domainname = tomoyo_get_name(domainname);
		if (!e.domainname)
			goto out;
	}
	e.program = tomoyo_get_name(program);
	if (!e.program)
		goto out;
	if (mutex_lock_interruptible(&tomoyo_policy_lock))
		goto out;
	list_for_each_entry_rcu(ptr, &tomoyo_domain_initializer_list, list) {
		if (!tomoyo_is_same_domain_initializer_entry(ptr, &e))
			continue;
		ptr->is_deleted = is_delete;
		error = 0;
		break;
	}
	if (!is_delete && error) {
		struct tomoyo_domain_initializer_entry *entry =
			tomoyo_commit_ok(&e, sizeof(e));
		if (entry) {
			list_add_tail_rcu(&entry->list,
					  &tomoyo_domain_initializer_list);
			error = 0;
		}
	}
	mutex_unlock(&tomoyo_policy_lock);
 out:
	tomoyo_put_name(e.domainname);
	tomoyo_put_name(e.program);
	return error;
}

bool tomoyo_read_domain_initializer_policy(struct tomoyo_io_buffer *head)
{
	struct list_head *pos;
	bool done = true;

	list_for_each_cookie(pos, head->read_var2,
			     &tomoyo_domain_initializer_list) {
		const char *no;
		const char *from = "";
		const char *domain = "";
		struct tomoyo_domain_initializer_entry *ptr;
		ptr = list_entry(pos, struct tomoyo_domain_initializer_entry,
				  list);
		if (ptr->is_deleted)
			continue;
		no = ptr->is_not ? "no_" : "";
		if (ptr->domainname) {
			from = " from ";
			domain = ptr->domainname->name;
		}
		done = tomoyo_io_printf(head,
					"%s" TOMOYO_KEYWORD_INITIALIZE_DOMAIN
					"%s%s%s\n", no, ptr->program->name,
					from, domain);
		if (!done)
			break;
	}
	return done;
}

int tomoyo_write_domain_initializer_policy(char *data, const bool is_not,
					   const bool is_delete)
{
	char *cp = strstr(data, " from ");

	if (cp) {
		*cp = '\0';
		return tomoyo_update_domain_initializer_entry(cp + 6, data,
							      is_not,
							      is_delete);
	}
	return tomoyo_update_domain_initializer_entry(NULL, data, is_not,
						      is_delete);
}

static bool tomoyo_is_domain_initializer(const struct tomoyo_path_info *
					 domainname,
					 const struct tomoyo_path_info *program,
					 const struct tomoyo_path_info *
					 last_name)
{
	struct tomoyo_domain_initializer_entry *ptr;
	bool flag = false;

	list_for_each_entry_rcu(ptr, &tomoyo_domain_initializer_list, list) {
		if (ptr->is_deleted)
			continue;
		if (ptr->domainname) {
			if (!ptr->is_last_name) {
				if (ptr->domainname != domainname)
					continue;
			} else {
				if (tomoyo_pathcmp(ptr->domainname, last_name))
					continue;
			}
		}
		if (tomoyo_pathcmp(ptr->program, program))
			continue;
		if (ptr->is_not) {
			flag = false;
			break;
		}
		flag = true;
	}
	return flag;
}

LIST_HEAD(tomoyo_domain_keeper_list);

static int tomoyo_update_domain_keeper_entry(const char *domainname,
					     const char *program,
					     const bool is_not,
					     const bool is_delete)
{
	struct tomoyo_domain_keeper_entry *ptr;
	struct tomoyo_domain_keeper_entry e = { .is_not = is_not };
	int error = is_delete ? -ENOENT : -ENOMEM;

	if (!tomoyo_is_domain_def(domainname) &&
	    tomoyo_is_correct_path(domainname, 1, -1, -1))
		e.is_last_name = true;
	else if (!tomoyo_is_correct_domain(domainname))
		return -EINVAL;
	if (program) {
		if (!tomoyo_is_correct_path(program, 1, -1, -1))
			return -EINVAL;
		e.program = tomoyo_get_name(program);
		if (!e.program)
			goto out;
	}
	e.domainname = tomoyo_get_name(domainname);
	if (!e.domainname)
		goto out;
	if (mutex_lock_interruptible(&tomoyo_policy_lock))
		goto out;
	list_for_each_entry_rcu(ptr, &tomoyo_domain_keeper_list, list) {
		if (!tomoyo_is_same_domain_keeper_entry(ptr, &e))
			continue;
		ptr->is_deleted = is_delete;
		error = 0;
		break;
	}
	if (!is_delete && error) {
		struct tomoyo_domain_keeper_entry *entry =
			tomoyo_commit_ok(&e, sizeof(e));
		if (entry) {
			list_add_tail_rcu(&entry->list,
					  &tomoyo_domain_keeper_list);
			error = 0;
		}
	}
	mutex_unlock(&tomoyo_policy_lock);
 out:
	tomoyo_put_name(e.domainname);
	tomoyo_put_name(e.program);
	return error;
}

int tomoyo_write_domain_keeper_policy(char *data, const bool is_not,
				      const bool is_delete)
{
	char *cp = strstr(data, " from ");

	if (cp) {
		*cp = '\0';
		return tomoyo_update_domain_keeper_entry(cp + 6, data, is_not,
							 is_delete);
	}
	return tomoyo_update_domain_keeper_entry(data, NULL, is_not, is_delete);
}

bool tomoyo_read_domain_keeper_policy(struct tomoyo_io_buffer *head)
{
	struct list_head *pos;
	bool done = true;

	list_for_each_cookie(pos, head->read_var2,
			     &tomoyo_domain_keeper_list) {
		struct tomoyo_domain_keeper_entry *ptr;
		const char *no;
		const char *from = "";
		const char *program = "";

		ptr = list_entry(pos, struct tomoyo_domain_keeper_entry, list);
		if (ptr->is_deleted)
			continue;
		no = ptr->is_not ? "no_" : "";
		if (ptr->program) {
			from = " from ";
			program = ptr->program->name;
		}
		done = tomoyo_io_printf(head,
					"%s" TOMOYO_KEYWORD_KEEP_DOMAIN
					"%s%s%s\n", no, program, from,
					ptr->domainname->name);
		if (!done)
			break;
	}
	return done;
}

static bool tomoyo_is_domain_keeper(const struct tomoyo_path_info *domainname,
				    const struct tomoyo_path_info *program,
				    const struct tomoyo_path_info *last_name)
{
	struct tomoyo_domain_keeper_entry *ptr;
	bool flag = false;

	list_for_each_entry_rcu(ptr, &tomoyo_domain_keeper_list, list) {
		if (ptr->is_deleted)
			continue;
		if (!ptr->is_last_name) {
			if (ptr->domainname != domainname)
				continue;
		} else {
			if (tomoyo_pathcmp(ptr->domainname, last_name))
				continue;
		}
		if (ptr->program && tomoyo_pathcmp(ptr->program, program))
			continue;
		if (ptr->is_not) {
			flag = false;
			break;
		}
		flag = true;
	}
	return flag;
}

LIST_HEAD(tomoyo_alias_list);

static int tomoyo_update_alias_entry(const char *original_name,
				     const char *aliased_name,
				     const bool is_delete)
{
	struct tomoyo_alias_entry *ptr;
	struct tomoyo_alias_entry e = { };
	int error = is_delete ? -ENOENT : -ENOMEM;

	if (!tomoyo_is_correct_path(original_name, 1, -1, -1) ||
	    !tomoyo_is_correct_path(aliased_name, 1, -1, -1))
		return -EINVAL; /* No patterns allowed. */
	e.original_name = tomoyo_get_name(original_name);
	e.aliased_name = tomoyo_get_name(aliased_name);
	if (!e.original_name || !e.aliased_name)
		goto out;
	if (mutex_lock_interruptible(&tomoyo_policy_lock))
		goto out;
	list_for_each_entry_rcu(ptr, &tomoyo_alias_list, list) {
		if (!tomoyo_is_same_alias_entry(ptr, &e))
			continue;
		ptr->is_deleted = is_delete;
		error = 0;
		break;
	}
	if (!is_delete && error) {
		struct tomoyo_alias_entry *entry =
			tomoyo_commit_ok(&e, sizeof(e));
		if (entry) {
			list_add_tail_rcu(&entry->list, &tomoyo_alias_list);
			error = 0;
		}
	}
	mutex_unlock(&tomoyo_policy_lock);
 out:
	tomoyo_put_name(e.original_name);
	tomoyo_put_name(e.aliased_name);
	return error;
}

bool tomoyo_read_alias_policy(struct tomoyo_io_buffer *head)
{
	struct list_head *pos;
	bool done = true;

	list_for_each_cookie(pos, head->read_var2, &tomoyo_alias_list) {
		struct tomoyo_alias_entry *ptr;

		ptr = list_entry(pos, struct tomoyo_alias_entry, list);
		if (ptr->is_deleted)
			continue;
		done = tomoyo_io_printf(head, TOMOYO_KEYWORD_ALIAS "%s %s\n",
					ptr->original_name->name,
					ptr->aliased_name->name);
		if (!done)
			break;
	}
	return done;
}

int tomoyo_write_alias_policy(char *data, const bool is_delete)
{
	char *cp = strchr(data, ' ');

	if (!cp)
		return -EINVAL;
	*cp++ = '\0';
	return tomoyo_update_alias_entry(data, cp, is_delete);
}

struct tomoyo_domain_info *tomoyo_find_or_assign_new_domain(const char *
							    domainname,
							    const u8 profile)
{
	struct tomoyo_domain_info *entry;
	struct tomoyo_domain_info *domain = NULL;
	const struct tomoyo_path_info *saved_domainname;
	bool found = false;

	if (!tomoyo_is_correct_domain(domainname))
		return NULL;
	saved_domainname = tomoyo_get_name(domainname);
	if (!saved_domainname)
		return NULL;
	entry = kzalloc(sizeof(*entry), GFP_NOFS);
	if (mutex_lock_interruptible(&tomoyo_policy_lock))
		goto out;
	list_for_each_entry_rcu(domain, &tomoyo_domain_list, list) {
		if (domain->is_deleted ||
		    tomoyo_pathcmp(saved_domainname, domain->domainname))
			continue;
		found = true;
		break;
	}
	if (!found && tomoyo_memory_ok(entry)) {
		INIT_LIST_HEAD(&entry->acl_info_list);
		entry->domainname = saved_domainname;
		saved_domainname = NULL;
		entry->profile = profile;
		list_add_tail_rcu(&entry->list, &tomoyo_domain_list);
		domain = entry;
		entry = NULL;
		found = true;
	}
	mutex_unlock(&tomoyo_policy_lock);
 out:
	tomoyo_put_name(saved_domainname);
	kfree(entry);
	return found ? domain : NULL;
}

int tomoyo_find_next_domain(struct linux_binprm *bprm)
{
	/*
	 * This function assumes that the size of buffer returned by
	 * tomoyo_realpath() = TOMOYO_MAX_PATHNAME_LEN.
	 */
	struct tomoyo_page_buffer *tmp = kzalloc(sizeof(*tmp), GFP_NOFS);
	struct tomoyo_domain_info *old_domain = tomoyo_domain();
	struct tomoyo_domain_info *domain = NULL;
	const char *old_domain_name = old_domain->domainname->name;
	const char *original_name = bprm->filename;
	char *new_domain_name = NULL;
	char *real_program_name = NULL;
	char *symlink_program_name = NULL;
	const u8 mode = tomoyo_check_flags(old_domain, TOMOYO_MAC_FOR_FILE);
	const bool is_enforce = (mode == 3);
	int retval = -ENOMEM;
	struct tomoyo_path_info r; /* real name */
	struct tomoyo_path_info s; /* symlink name */
	struct tomoyo_path_info l; /* last name */
	static bool initialized;

	if (!tmp)
		goto out;

	if (!initialized) {
		/*
		 * Built-in initializers. This is needed because policies are
		 * not loaded until starting /sbin/init.
		 */
		tomoyo_update_domain_initializer_entry(NULL, "/sbin/hotplug",
						       false, false);
		tomoyo_update_domain_initializer_entry(NULL, "/sbin/modprobe",
						       false, false);
		initialized = true;
	}

	/* Get tomoyo_realpath of program. */
	retval = -ENOENT;
	/* I hope tomoyo_realpath() won't fail with -ENOMEM. */
	real_program_name = tomoyo_realpath(original_name);
	if (!real_program_name)
		goto out;
	/* Get tomoyo_realpath of symbolic link. */
	symlink_program_name = tomoyo_realpath_nofollow(original_name);
	if (!symlink_program_name)
		goto out;

	r.name = real_program_name;
	tomoyo_fill_path_info(&r);
	s.name = symlink_program_name;
	tomoyo_fill_path_info(&s);
	l.name = tomoyo_get_last_name(old_domain);
	tomoyo_fill_path_info(&l);

	/* Check 'alias' directive. */
	if (tomoyo_pathcmp(&r, &s)) {
		struct tomoyo_alias_entry *ptr;
		/* Is this program allowed to be called via symbolic links? */
		list_for_each_entry_rcu(ptr, &tomoyo_alias_list, list) {
			if (ptr->is_deleted ||
			    tomoyo_pathcmp(&r, ptr->original_name) ||
			    tomoyo_pathcmp(&s, ptr->aliased_name))
				continue;
			memset(real_program_name, 0, TOMOYO_MAX_PATHNAME_LEN);
			strncpy(real_program_name, ptr->aliased_name->name,
				TOMOYO_MAX_PATHNAME_LEN - 1);
			tomoyo_fill_path_info(&r);
			break;
		}
	}

	/* Check execute permission. */
	retval = tomoyo_check_exec_perm(old_domain, &r);
	if (retval < 0)
		goto out;

	new_domain_name = tmp->buffer;
	if (tomoyo_is_domain_initializer(old_domain->domainname, &r, &l)) {
		/* Transit to the child of tomoyo_kernel_domain domain. */
		snprintf(new_domain_name, TOMOYO_MAX_PATHNAME_LEN + 1,
			 TOMOYO_ROOT_NAME " " "%s", real_program_name);
	} else if (old_domain == &tomoyo_kernel_domain &&
		   !tomoyo_policy_loaded) {
		/*
		 * Needn't to transit from kernel domain before starting
		 * /sbin/init. But transit from kernel domain if executing
		 * initializers because they might start before /sbin/init.
		 */
		domain = old_domain;
	} else if (tomoyo_is_domain_keeper(old_domain->domainname, &r, &l)) {
		/* Keep current domain. */
		domain = old_domain;
	} else {
		/* Normal domain transition. */
		snprintf(new_domain_name, TOMOYO_MAX_PATHNAME_LEN + 1,
			 "%s %s", old_domain_name, real_program_name);
	}
	if (domain || strlen(new_domain_name) >= TOMOYO_MAX_PATHNAME_LEN)
		goto done;
	domain = tomoyo_find_domain(new_domain_name);
	if (domain)
		goto done;
	if (is_enforce)
		goto done;
	domain = tomoyo_find_or_assign_new_domain(new_domain_name,
						  old_domain->profile);
 done:
	if (domain)
		goto out;
	printk(KERN_WARNING "TOMOYO-ERROR: Domain '%s' not defined.\n",
	       new_domain_name);
	if (is_enforce)
		retval = -EPERM;
	else
		old_domain->transition_failed = true;
 out:
	if (!domain)
		domain = old_domain;
	/* Update reference count on "struct tomoyo_domain_info". */
	atomic_inc(&domain->users);
	bprm->cred->security = domain;
	kfree(real_program_name);
	kfree(symlink_program_name);
	kfree(tmp);
	return retval;
}

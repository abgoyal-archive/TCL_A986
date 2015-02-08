

#include <linux/types.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include "smack.h"

struct smack_known smack_known_huh = {
	.smk_known	= "?",
	.smk_secid	= 2,
	.smk_cipso	= NULL,
};

struct smack_known smack_known_hat = {
	.smk_known	= "^",
	.smk_secid	= 3,
	.smk_cipso	= NULL,
};

struct smack_known smack_known_star = {
	.smk_known	= "*",
	.smk_secid	= 4,
	.smk_cipso	= NULL,
};

struct smack_known smack_known_floor = {
	.smk_known	= "_",
	.smk_secid	= 5,
	.smk_cipso	= NULL,
};

struct smack_known smack_known_invalid = {
	.smk_known	= "",
	.smk_secid	= 6,
	.smk_cipso	= NULL,
};

struct smack_known smack_known_web = {
	.smk_known	= "@",
	.smk_secid	= 7,
	.smk_cipso	= NULL,
};

LIST_HEAD(smack_known_list);

static u32 smack_next_secid = 10;

int log_policy = SMACK_AUDIT_DENIED;

int smk_access(char *subject_label, char *object_label, int request,
	       struct smk_audit_info *a)
{
	u32 may = MAY_NOT;
	struct smack_rule *srp;
	int rc = 0;

	/*
	 * Hardcoded comparisons.
	 *
	 * A star subject can't access any object.
	 */
	if (subject_label == smack_known_star.smk_known ||
	    strcmp(subject_label, smack_known_star.smk_known) == 0) {
		rc = -EACCES;
		goto out_audit;
	}
	/*
	 * An internet object can be accessed by any subject.
	 * Tasks cannot be assigned the internet label.
	 * An internet subject can access any object.
	 */
	if (object_label == smack_known_web.smk_known ||
	    subject_label == smack_known_web.smk_known ||
	    strcmp(object_label, smack_known_web.smk_known) == 0 ||
	    strcmp(subject_label, smack_known_web.smk_known) == 0)
		goto out_audit;
	/*
	 * A star object can be accessed by any subject.
	 */
	if (object_label == smack_known_star.smk_known ||
	    strcmp(object_label, smack_known_star.smk_known) == 0)
		goto out_audit;
	/*
	 * An object can be accessed in any way by a subject
	 * with the same label.
	 */
	if (subject_label == object_label ||
	    strcmp(subject_label, object_label) == 0)
		goto out_audit;
	/*
	 * A hat subject can read any object.
	 * A floor object can be read by any subject.
	 */
	if ((request & MAY_ANYREAD) == request) {
		if (object_label == smack_known_floor.smk_known ||
		    strcmp(object_label, smack_known_floor.smk_known) == 0)
			goto out_audit;
		if (subject_label == smack_known_hat.smk_known ||
		    strcmp(subject_label, smack_known_hat.smk_known) == 0)
			goto out_audit;
	}
	/*
	 * Beyond here an explicit relationship is required.
	 * If the requested access is contained in the available
	 * access (e.g. read is included in readwrite) it's
	 * good.
	 */
	rcu_read_lock();
	list_for_each_entry_rcu(srp, &smack_rule_list, list) {
		if (srp->smk_subject == subject_label ||
		    strcmp(srp->smk_subject, subject_label) == 0) {
			if (srp->smk_object == object_label ||
			    strcmp(srp->smk_object, object_label) == 0) {
				may = srp->smk_access;
				break;
			}
		}
	}
	rcu_read_unlock();
	/*
	 * This is a bit map operation.
	 */
	if ((request & may) == request)
		goto out_audit;

	rc = -EACCES;
out_audit:
#ifdef CONFIG_AUDIT
	if (a)
		smack_log(subject_label, object_label, request, rc, a);
#endif
	return rc;
}

int smk_curacc(char *obj_label, u32 mode, struct smk_audit_info *a)
{
	int rc;
	char *sp = current_security();

	rc = smk_access(sp, obj_label, mode, NULL);
	if (rc == 0)
		goto out_audit;

	/*
	 * Return if a specific label has been designated as the
	 * only one that gets privilege and current does not
	 * have that label.
	 */
	if (smack_onlycap != NULL && smack_onlycap != current->cred->security)
		goto out_audit;

	if (capable(CAP_MAC_OVERRIDE))
		return 0;

out_audit:
#ifdef CONFIG_AUDIT
	if (a)
		smack_log(sp, obj_label, mode, rc, a);
#endif
	return rc;
}

#ifdef CONFIG_AUDIT
static inline void smack_str_from_perm(char *string, int access)
{
	int i = 0;
	if (access & MAY_READ)
		string[i++] = 'r';
	if (access & MAY_WRITE)
		string[i++] = 'w';
	if (access & MAY_EXEC)
		string[i++] = 'x';
	if (access & MAY_APPEND)
		string[i++] = 'a';
	string[i] = '\0';
}
static void smack_log_callback(struct audit_buffer *ab, void *a)
{
	struct common_audit_data *ad = a;
	struct smack_audit_data *sad = &ad->smack_audit_data;
	audit_log_format(ab, "lsm=SMACK fn=%s action=%s",
			 ad->smack_audit_data.function,
			 sad->result ? "denied" : "granted");
	audit_log_format(ab, " subject=");
	audit_log_untrustedstring(ab, sad->subject);
	audit_log_format(ab, " object=");
	audit_log_untrustedstring(ab, sad->object);
	audit_log_format(ab, " requested=%s", sad->request);
}

void smack_log(char *subject_label, char *object_label, int request,
	       int result, struct smk_audit_info *ad)
{
	char request_buffer[SMK_NUM_ACCESS_TYPE + 1];
	struct smack_audit_data *sad;
	struct common_audit_data *a = &ad->a;

	/* check if we have to log the current event */
	if (result != 0 && (log_policy & SMACK_AUDIT_DENIED) == 0)
		return;
	if (result == 0 && (log_policy & SMACK_AUDIT_ACCEPT) == 0)
		return;

	if (a->smack_audit_data.function == NULL)
		a->smack_audit_data.function = "unknown";

	/* end preparing the audit data */
	sad = &a->smack_audit_data;
	smack_str_from_perm(request_buffer, request);
	sad->subject = subject_label;
	sad->object  = object_label;
	sad->request = request_buffer;
	sad->result  = result;
	a->lsm_pre_audit = smack_log_callback;

	common_lsm_audit(a);
}
#else /* #ifdef CONFIG_AUDIT */
void smack_log(char *subject_label, char *object_label, int request,
               int result, struct smk_audit_info *ad)
{
}
#endif

static DEFINE_MUTEX(smack_known_lock);

struct smack_known *smk_import_entry(const char *string, int len)
{
	struct smack_known *skp;
	char smack[SMK_LABELLEN];
	int found;
	int i;

	if (len <= 0 || len > SMK_MAXLEN)
		len = SMK_MAXLEN;

	for (i = 0, found = 0; i < SMK_LABELLEN; i++) {
		if (found)
			smack[i] = '\0';
		else if (i >= len || string[i] > '~' || string[i] <= ' ' ||
			 string[i] == '/' || string[i] == '"' ||
			 string[i] == '\\' || string[i] == '\'') {
			smack[i] = '\0';
			found = 1;
		} else
			smack[i] = string[i];
	}

	if (smack[0] == '\0')
		return NULL;

	mutex_lock(&smack_known_lock);

	found = 0;
	list_for_each_entry_rcu(skp, &smack_known_list, list) {
		if (strncmp(skp->smk_known, smack, SMK_MAXLEN) == 0) {
			found = 1;
			break;
		}
	}

	if (found == 0) {
		skp = kzalloc(sizeof(struct smack_known), GFP_KERNEL);
		if (skp != NULL) {
			strncpy(skp->smk_known, smack, SMK_MAXLEN);
			skp->smk_secid = smack_next_secid++;
			skp->smk_cipso = NULL;
			spin_lock_init(&skp->smk_cipsolock);
			/*
			 * Make sure that the entry is actually
			 * filled before putting it on the list.
			 */
			list_add_rcu(&skp->list, &smack_known_list);
		}
	}

	mutex_unlock(&smack_known_lock);

	return skp;
}

char *smk_import(const char *string, int len)
{
	struct smack_known *skp;

	/* labels cannot begin with a '-' */
	if (string[0] == '-')
		return NULL;
	skp = smk_import_entry(string, len);
	if (skp == NULL)
		return NULL;
	return skp->smk_known;
}

char *smack_from_secid(const u32 secid)
{
	struct smack_known *skp;

	rcu_read_lock();
	list_for_each_entry_rcu(skp, &smack_known_list, list) {
		if (skp->smk_secid == secid) {
			rcu_read_unlock();
			return skp->smk_known;
		}
	}

	/*
	 * If we got this far someone asked for the translation
	 * of a secid that is not on the list.
	 */
	rcu_read_unlock();
	return smack_known_invalid.smk_known;
}

u32 smack_to_secid(const char *smack)
{
	struct smack_known *skp;

	rcu_read_lock();
	list_for_each_entry_rcu(skp, &smack_known_list, list) {
		if (strncmp(skp->smk_known, smack, SMK_MAXLEN) == 0) {
			rcu_read_unlock();
			return skp->smk_secid;
		}
	}
	rcu_read_unlock();
	return 0;
}

void smack_from_cipso(u32 level, char *cp, char *result)
{
	struct smack_known *kp;
	char *final = NULL;

	rcu_read_lock();
	list_for_each_entry(kp, &smack_known_list, list) {
		if (kp->smk_cipso == NULL)
			continue;

		spin_lock_bh(&kp->smk_cipsolock);

		if (kp->smk_cipso->smk_level == level &&
		    memcmp(kp->smk_cipso->smk_catset, cp, SMK_LABELLEN) == 0)
			final = kp->smk_known;

		spin_unlock_bh(&kp->smk_cipsolock);
	}
	rcu_read_unlock();
	if (final == NULL)
		final = smack_known_huh.smk_known;
	strncpy(result, final, SMK_MAXLEN);
	return;
}

int smack_to_cipso(const char *smack, struct smack_cipso *cp)
{
	struct smack_known *kp;
	int found = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(kp, &smack_known_list, list) {
		if (kp->smk_known == smack ||
		    strcmp(kp->smk_known, smack) == 0) {
			found = 1;
			break;
		}
	}
	rcu_read_unlock();

	if (found == 0 || kp->smk_cipso == NULL)
		return -ENOENT;

	memcpy(cp, kp->smk_cipso, sizeof(struct smack_cipso));
	return 0;
}

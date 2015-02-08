


#include "qemu-common.h"
#include "sysemu.h"
#include "acl.h"

#ifdef HAVE_FNMATCH_H
#include <fnmatch.h>
#endif


static unsigned int nacls = 0;
static qemu_acl **acls = NULL;



qemu_acl *qemu_acl_find(const char *aclname)
{
    int i;
    for (i = 0 ; i < nacls ; i++) {
        if (strcmp(acls[i]->aclname, aclname) == 0)
            return acls[i];
    }

    return NULL;
}

qemu_acl *qemu_acl_init(const char *aclname)
{
    qemu_acl *acl;

    acl = qemu_acl_find(aclname);
    if (acl)
        return acl;

    acl = qemu_malloc(sizeof(*acl));
    acl->aclname = qemu_strdup(aclname);
    /* Deny by default, so there is no window of "open
     * access" between QEMU starting, and the user setting
     * up ACLs in the monitor */
    acl->defaultDeny = 1;

    acl->nentries = 0;
    QTAILQ_INIT(&acl->entries);

    acls = qemu_realloc(acls, sizeof(*acls) * (nacls +1));
    acls[nacls] = acl;
    nacls++;

    return acl;
}

int qemu_acl_party_is_allowed(qemu_acl *acl,
                              const char *party)
{
    qemu_acl_entry *entry;

    QTAILQ_FOREACH(entry, &acl->entries, next) {
#ifdef HAVE_FNMATCH_H
        if (fnmatch(entry->match, party, 0) == 0)
            return entry->deny ? 0 : 1;
#else
        /* No fnmatch, so fallback to exact string matching
         * instead of allowing wildcards */
        if (strcmp(entry->match, party) == 0)
            return entry->deny ? 0 : 1;
#endif
    }

    return acl->defaultDeny ? 0 : 1;
}


void qemu_acl_reset(qemu_acl *acl)
{
    qemu_acl_entry *entry;

    /* Put back to deny by default, so there is no window
     * of "open access" while the user re-initializes the
     * access control list */
    acl->defaultDeny = 1;
    QTAILQ_FOREACH(entry, &acl->entries, next) {
        QTAILQ_REMOVE(&acl->entries, entry, next);
        free(entry->match);
        free(entry);
    }
    acl->nentries = 0;
}


int qemu_acl_append(qemu_acl *acl,
                    int deny,
                    const char *match)
{
    qemu_acl_entry *entry;

    entry = qemu_malloc(sizeof(*entry));
    entry->match = qemu_strdup(match);
    entry->deny = deny;

    QTAILQ_INSERT_TAIL(&acl->entries, entry, next);
    acl->nentries++;

    return acl->nentries;
}


int qemu_acl_insert(qemu_acl *acl,
                    int deny,
                    const char *match,
                    int index)
{
    qemu_acl_entry *entry;
    qemu_acl_entry *tmp;
    int i = 0;

    if (index <= 0)
        return -1;
    if (index >= acl->nentries)
        return qemu_acl_append(acl, deny, match);


    entry = qemu_malloc(sizeof(*entry));
    entry->match = qemu_strdup(match);
    entry->deny = deny;

    QTAILQ_FOREACH(tmp, &acl->entries, next) {
        i++;
        if (i == index) {
            QTAILQ_INSERT_BEFORE(tmp, entry, next);
            acl->nentries++;
            break;
        }
    }

    return i;
}

int qemu_acl_remove(qemu_acl *acl,
                    const char *match)
{
    qemu_acl_entry *entry;
    int i = 0;

    QTAILQ_FOREACH(entry, &acl->entries, next) {
        i++;
        if (strcmp(entry->match, match) == 0) {
            QTAILQ_REMOVE(&acl->entries, entry, next);
            return i;
        }
    }
    return -1;
}





#include "core.h"
#include "user_reg.h"



struct tipc_user {
	int next;
	tipc_mode_event callback;
	void *usr_handle;
	struct list_head ports;
};

#define MAX_USERID 64
#define USER_LIST_SIZE ((MAX_USERID + 1) * sizeof(struct tipc_user))

static struct tipc_user *users = NULL;
static u32 next_free_user = MAX_USERID + 1;
static DEFINE_SPINLOCK(reg_lock);


static int reg_init(void)
{
	u32 i;

	spin_lock_bh(&reg_lock);
	if (!users) {
		users = kzalloc(USER_LIST_SIZE, GFP_ATOMIC);
		if (users) {
			for (i = 1; i <= MAX_USERID; i++) {
				users[i].next = i - 1;
			}
			next_free_user = MAX_USERID;
		}
	}
	spin_unlock_bh(&reg_lock);
	return users ? 0 : -ENOMEM;
}


static void reg_callback(struct tipc_user *user_ptr)
{
	tipc_mode_event cb;
	void *arg;

	spin_lock_bh(&reg_lock);
	cb = user_ptr->callback;
	arg = user_ptr->usr_handle;
	spin_unlock_bh(&reg_lock);

	if (cb)
		cb(arg, tipc_mode, tipc_own_addr);
}


int tipc_reg_start(void)
{
	u32 u;
	int res;

	if ((res = reg_init()))
		return res;

	for (u = 1; u <= MAX_USERID; u++) {
		if (users[u].callback)
			tipc_k_signal((Handler)reg_callback,
				      (unsigned long)&users[u]);
	}
	return 0;
}


void tipc_reg_stop(void)
{
	int id;

	if (!users)
		return;

	for (id = 1; id <= MAX_USERID; id++) {
		if (users[id].callback)
			reg_callback(&users[id]);
	}
	kfree(users);
	users = NULL;
}


int tipc_attach(u32 *userid, tipc_mode_event cb, void *usr_handle)
{
	struct tipc_user *user_ptr;

	if ((tipc_mode == TIPC_NOT_RUNNING) && !cb)
		return -ENOPROTOOPT;
	if (!users)
		reg_init();

	spin_lock_bh(&reg_lock);
	if (!next_free_user) {
		spin_unlock_bh(&reg_lock);
		return -EBUSY;
	}
	user_ptr = &users[next_free_user];
	*userid = next_free_user;
	next_free_user = user_ptr->next;
	user_ptr->next = -1;
	spin_unlock_bh(&reg_lock);

	user_ptr->callback = cb;
	user_ptr->usr_handle = usr_handle;
	INIT_LIST_HEAD(&user_ptr->ports);
	atomic_inc(&tipc_user_count);

	if (cb && (tipc_mode != TIPC_NOT_RUNNING))
		tipc_k_signal((Handler)reg_callback, (unsigned long)user_ptr);
	return 0;
}


void tipc_detach(u32 userid)
{
	struct tipc_user *user_ptr;
	struct list_head ports_temp;
	struct user_port *up_ptr, *temp_up_ptr;

	if ((userid == 0) || (userid > MAX_USERID))
		return;

	spin_lock_bh(&reg_lock);
	if ((!users) || (users[userid].next >= 0)) {
		spin_unlock_bh(&reg_lock);
		return;
	}

	user_ptr = &users[userid];
	user_ptr->callback = NULL;
	INIT_LIST_HEAD(&ports_temp);
	list_splice(&user_ptr->ports, &ports_temp);
	user_ptr->next = next_free_user;
	next_free_user = userid;
	spin_unlock_bh(&reg_lock);

	atomic_dec(&tipc_user_count);

	list_for_each_entry_safe(up_ptr, temp_up_ptr, &ports_temp, uport_list) {
		tipc_deleteport(up_ptr->ref);
	}
}


int tipc_reg_add_port(struct user_port *up_ptr)
{
	struct tipc_user *user_ptr;

	if (up_ptr->user_ref == 0)
		return 0;
	if (up_ptr->user_ref > MAX_USERID)
		return -EINVAL;
	if ((tipc_mode == TIPC_NOT_RUNNING) || !users )
		return -ENOPROTOOPT;

	spin_lock_bh(&reg_lock);
	user_ptr = &users[up_ptr->user_ref];
	list_add(&up_ptr->uport_list, &user_ptr->ports);
	spin_unlock_bh(&reg_lock);
	return 0;
}


int tipc_reg_remove_port(struct user_port *up_ptr)
{
	if (up_ptr->user_ref == 0)
		return 0;
	if (up_ptr->user_ref > MAX_USERID)
		return -EINVAL;
	if (!users )
		return -ENOPROTOOPT;

	spin_lock_bh(&reg_lock);
	list_del_init(&up_ptr->uport_list);
	spin_unlock_bh(&reg_lock);
	return 0;
}


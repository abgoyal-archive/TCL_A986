


#include <bfa_os_inc.h>
#include <cs/bfa_log.h>

struct bfa_log_info_s {
	u32        start_idx;	/*  start index for a module */
	u32        total_count;	/*  total count for a module */
	enum bfa_log_severity level;	/*  global log level */
	bfa_log_cb_t	cbfn;		/*  callback function */
};

static struct bfa_log_info_s bfa_log_info[BFA_LOG_MODULE_ID_MAX + 1];
static u32 bfa_log_msg_total_count;
static int      bfa_log_initialized;

static char    *bfa_log_severity[] =
	{ "[none]", "[critical]", "[error]", "[warn]", "[info]", "" };

int
bfa_log_init(struct bfa_log_mod_s *log_mod, char *instance_name,
			bfa_log_cb_t cbfn)
{
	struct bfa_log_msgdef_s *msg;
	u32        pre_mod_id = 0;
	u32        cur_mod_id = 0;
	u32        i, pre_idx, idx, msg_id;

	/*
	 * set instance name
	 */
	if (log_mod) {
		strncpy(log_mod->instance_info, instance_name,
			sizeof(log_mod->instance_info));
		log_mod->cbfn = cbfn;
		for (i = 0; i <= BFA_LOG_MODULE_ID_MAX; i++)
			log_mod->log_level[i] = BFA_LOG_WARNING;
	}

	if (bfa_log_initialized)
		return 0;

	for (i = 0; i <= BFA_LOG_MODULE_ID_MAX; i++) {
		bfa_log_info[i].start_idx = 0;
		bfa_log_info[i].total_count = 0;
		bfa_log_info[i].level = BFA_LOG_WARNING;
		bfa_log_info[i].cbfn = cbfn;
	}

	pre_idx = 0;
	idx = 0;
	msg = bfa_log_msg_array;
	msg_id = BFA_LOG_GET_MSG_ID(msg);
	pre_mod_id = BFA_LOG_GET_MOD_ID(msg_id);
	while (msg_id != 0) {
		cur_mod_id = BFA_LOG_GET_MOD_ID(msg_id);

		if (cur_mod_id > BFA_LOG_MODULE_ID_MAX) {
			cbfn(log_mod, msg_id,
				"%s%s log: module id %u out of range\n",
				BFA_LOG_CAT_NAME,
				bfa_log_severity[BFA_LOG_ERROR],
				cur_mod_id);
			return -1;
		}

		if (pre_mod_id > BFA_LOG_MODULE_ID_MAX) {
			cbfn(log_mod, msg_id,
				"%s%s log: module id %u out of range\n",
				BFA_LOG_CAT_NAME,
				bfa_log_severity[BFA_LOG_ERROR],
				pre_mod_id);
			return -1;
		}

		if (cur_mod_id != pre_mod_id) {
			bfa_log_info[pre_mod_id].start_idx = pre_idx;
			bfa_log_info[pre_mod_id].total_count = idx - pre_idx;
			pre_mod_id = cur_mod_id;
			pre_idx = idx;
		}

		idx++;
		msg++;
		msg_id = BFA_LOG_GET_MSG_ID(msg);
	}

	bfa_log_info[cur_mod_id].start_idx = pre_idx;
	bfa_log_info[cur_mod_id].total_count = idx - pre_idx;
	bfa_log_msg_total_count = idx;

	cbfn(log_mod, msg_id, "%s%s log: init OK, msg total count %u\n",
		BFA_LOG_CAT_NAME,
		bfa_log_severity[BFA_LOG_INFO], bfa_log_msg_total_count);

	bfa_log_initialized = 1;

	return 0;
}

bfa_status_t
bfa_log_set_level(struct bfa_log_mod_s *log_mod, int mod_id,
		  enum bfa_log_severity log_level)
{
	if (mod_id <= BFA_LOG_UNUSED_ID || mod_id > BFA_LOG_MODULE_ID_MAX)
		return BFA_STATUS_EINVAL;

	if (log_level <= BFA_LOG_INVALID || log_level > BFA_LOG_LEVEL_MAX)
		return BFA_STATUS_EINVAL;

	if (log_mod)
		log_mod->log_level[mod_id] = log_level;
	else
		bfa_log_info[mod_id].level = log_level;

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_log_set_level_all(struct bfa_log_mod_s *log_mod,
		  enum bfa_log_severity log_level)
{
	int mod_id = BFA_LOG_UNUSED_ID + 1;

	if (log_level <= BFA_LOG_INVALID || log_level > BFA_LOG_LEVEL_MAX)
		return BFA_STATUS_EINVAL;

	if (log_mod) {
		for (; mod_id <= BFA_LOG_MODULE_ID_MAX; mod_id++)
			log_mod->log_level[mod_id] = log_level;
	} else {
		for (; mod_id <= BFA_LOG_MODULE_ID_MAX; mod_id++)
			bfa_log_info[mod_id].level = log_level;
	}

	return BFA_STATUS_OK;
}

bfa_status_t
bfa_log_set_level_aen(struct bfa_log_mod_s *log_mod,
		  enum bfa_log_severity log_level)
{
	int mod_id = BFA_LOG_AEN_MIN + 1;

	if (log_mod) {
		for (; mod_id <= BFA_LOG_AEN_MAX; mod_id++)
			log_mod->log_level[mod_id] = log_level;
	} else {
		for (; mod_id <= BFA_LOG_AEN_MAX; mod_id++)
			bfa_log_info[mod_id].level = log_level;
	}

	return BFA_STATUS_OK;
}

enum bfa_log_severity
bfa_log_get_level(struct bfa_log_mod_s *log_mod, int mod_id)
{
	if (mod_id <= BFA_LOG_UNUSED_ID || mod_id > BFA_LOG_MODULE_ID_MAX)
		return BFA_LOG_INVALID;

	if (log_mod)
		return log_mod->log_level[mod_id];
	else
		return bfa_log_info[mod_id].level;
}

enum bfa_log_severity
bfa_log_get_msg_level(struct bfa_log_mod_s *log_mod, u32 msg_id)
{
	struct bfa_log_msgdef_s *msg;
	u32        mod = BFA_LOG_GET_MOD_ID(msg_id);
	u32        idx = BFA_LOG_GET_MSG_IDX(msg_id) - 1;

	if (!bfa_log_initialized)
		return BFA_LOG_INVALID;

	if (mod > BFA_LOG_MODULE_ID_MAX)
		return BFA_LOG_INVALID;

	if (idx >= bfa_log_info[mod].total_count) {
		bfa_log_info[mod].cbfn(log_mod, msg_id,
			"%s%s log: inconsistent idx %u vs. total count %u\n",
			BFA_LOG_CAT_NAME, bfa_log_severity[BFA_LOG_ERROR], idx,
			bfa_log_info[mod].total_count);
		return BFA_LOG_INVALID;
	}

	msg = bfa_log_msg_array + bfa_log_info[mod].start_idx + idx;
	if (msg_id != BFA_LOG_GET_MSG_ID(msg)) {
		bfa_log_info[mod].cbfn(log_mod, msg_id,
			"%s%s log: inconsistent msg id %u array msg id %u\n",
			BFA_LOG_CAT_NAME, bfa_log_severity[BFA_LOG_ERROR],
			msg_id, BFA_LOG_GET_MSG_ID(msg));
		return BFA_LOG_INVALID;
	}

	return BFA_LOG_GET_SEVERITY(msg);
}

int
bfa_log(struct bfa_log_mod_s *log_mod, u32 msg_id, ...)
{
	va_list         ap;
	char            buf[256];
	struct bfa_log_msgdef_s *msg;
	int             log_level;
	u32        mod = BFA_LOG_GET_MOD_ID(msg_id);
	u32        idx = BFA_LOG_GET_MSG_IDX(msg_id) - 1;

	if (!bfa_log_initialized)
		return -1;

	if (mod > BFA_LOG_MODULE_ID_MAX)
		return -1;

	if (idx >= bfa_log_info[mod].total_count) {
		bfa_log_info[mod].
			cbfn
			(log_mod, msg_id,
			"%s%s log: inconsistent idx %u vs. total count %u\n",
			BFA_LOG_CAT_NAME, bfa_log_severity[BFA_LOG_ERROR], idx,
			bfa_log_info[mod].total_count);
		return -1;
	}

	msg = bfa_log_msg_array + bfa_log_info[mod].start_idx + idx;
	if (msg_id != BFA_LOG_GET_MSG_ID(msg)) {
		bfa_log_info[mod].
			cbfn
			(log_mod, msg_id,
			"%s%s log: inconsistent msg id %u array msg id %u\n",
			BFA_LOG_CAT_NAME, bfa_log_severity[BFA_LOG_ERROR],
			msg_id, BFA_LOG_GET_MSG_ID(msg));
		return -1;
	}

	log_level = log_mod ? log_mod->log_level[mod] : bfa_log_info[mod].level;
	if ((BFA_LOG_GET_SEVERITY(msg) > log_level) &&
			(msg->attributes != BFA_LOG_ATTR_NONE))
		return 0;

	va_start(ap, msg_id);
	bfa_os_vsprintf(buf, BFA_LOG_GET_MSG_FMT_STRING(msg), ap);
	va_end(ap);

	if (log_mod)
		log_mod->cbfn(log_mod, msg_id, "%s[%s]%s%s %s: %s\n",
				BFA_LOG_CAT_NAME, log_mod->instance_info,
				bfa_log_severity[BFA_LOG_GET_SEVERITY(msg)],
				(msg->attributes & BFA_LOG_ATTR_AUDIT)
				? " (audit) " : "", msg->msg_value, buf);
	else
		bfa_log_info[mod].cbfn(log_mod, msg_id, "%s%s%s %s: %s\n",
				BFA_LOG_CAT_NAME,
				bfa_log_severity[BFA_LOG_GET_SEVERITY(msg)],
				(msg->attributes & BFA_LOG_ATTR_AUDIT) ?
				" (audit) " : "", msg->msg_value, buf);

	return 0;
}


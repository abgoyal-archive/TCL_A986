

#ifndef __DM_LOG_USERSPACE_H__
#define __DM_LOG_USERSPACE_H__

#include <linux/dm-ioctl.h> /* For DM_UUID_LEN */


#define DM_ULOG_CTR                    1

#define DM_ULOG_DTR                    2

#define DM_ULOG_PRESUSPEND             3

#define DM_ULOG_POSTSUSPEND            4

#define DM_ULOG_RESUME                 5

#define DM_ULOG_GET_REGION_SIZE        6

#define DM_ULOG_IS_CLEAN               7

#define DM_ULOG_IN_SYNC                8

#define DM_ULOG_FLUSH                  9

#define DM_ULOG_MARK_REGION           10

#define DM_ULOG_CLEAR_REGION          11

#define DM_ULOG_GET_RESYNC_WORK       12

#define DM_ULOG_SET_REGION_SYNC       13

#define DM_ULOG_GET_SYNC_COUNT        14

#define DM_ULOG_STATUS_INFO           15

#define DM_ULOG_STATUS_TABLE          16

#define DM_ULOG_IS_REMOTE_RECOVERING  17

#define DM_ULOG_REQUEST_MASK 0xFF
#define DM_ULOG_REQUEST_TYPE(request_type) \
	(DM_ULOG_REQUEST_MASK & (request_type))

struct dm_ulog_request {
	/*
	 * The local unique identifier (luid) and the universally unique
	 * identifier (uuid) are used to tie a request to a specific
	 * mirror log.  A single machine log could probably make due with
	 * just the 'luid', but a cluster-aware log must use the 'uuid' and
	 * the 'luid'.  The uuid is what is required for node to node
	 * communication concerning a particular log, but the 'luid' helps
	 * differentiate between logs that are being swapped and have the
	 * same 'uuid'.  (Think "live" and "inactive" device-mapper tables.)
	 */
	uint64_t luid;
	char uuid[DM_UUID_LEN];
	char padding[7];        /* Padding because DM_UUID_LEN = 129 */

	int32_t error;          /* Used to report back processing errors */

	uint32_t seq;           /* Sequence number for request */
	uint32_t request_type;  /* DM_ULOG_* defined above */
	uint32_t data_size;     /* How much data (not including this struct) */

	char data[0];
};

#endif /* __DM_LOG_USERSPACE_H__ */

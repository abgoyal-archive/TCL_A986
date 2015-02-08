

#include <linux/ioctl.h>
#include <linux/mutex.h>

#define VMCP_GETCODE _IOR(0x10, 1, int)
#define VMCP_SETBUF _IOW(0x10, 2, int)
#define VMCP_GETSIZE _IOR(0x10, 3, int)

struct vmcp_session {
	unsigned int bufsize;
	char *response;
	int resp_size;
	int resp_code;
	/* As we use copy_from/to_user, which might     *
	 * sleep and cannot use a spinlock              */
	struct mutex mutex;
};

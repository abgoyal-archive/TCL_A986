

#include <linux/uts.h>
#include <linux/version.h>
#include <linux/elfnote.h>

ELFNOTE_START(Linux, 0, "a")
	.long LINUX_VERSION_CODE
ELFNOTE_END

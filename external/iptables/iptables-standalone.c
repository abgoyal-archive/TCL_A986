

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <iptables.h>

#ifdef IPTABLES_MULTI
int
iptables_main(int argc, char *argv[])
#else
int
main(int argc, char *argv[])
#endif
{
	int ret;
	char *table = "filter";
	iptc_handle_t handle = NULL;

	program_name = "iptables";
	program_version = IPTABLES_VERSION;

	lib_dir = getenv("IPTABLES_LIB_DIR");
	if (!lib_dir)
		lib_dir = IPT_LIB_DIR;

#ifdef NO_SHARED_LIBS
	init_extensions();
#endif

	ret = do_command(argc, argv, &table, &handle);
	if (ret)
		ret = iptc_commit(&handle);

	if (!ret) {
		fprintf(stderr, "iptables: %s\n",
			iptc_strerror(errno));
		if (errno == EAGAIN) {
			exit(RESOURCE_PROBLEM);
		}
	}

	exit(!ret);
}

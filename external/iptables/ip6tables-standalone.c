

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ip6tables.h>

int
main(int argc, char *argv[])
{
	int ret;
	char *table = "filter";
	ip6tc_handle_t handle = NULL;

	program_name = "ip6tables";
	program_version = IPTABLES_VERSION;

	lib_dir = getenv("IP6TABLES_LIB_DIR");
	if (!lib_dir)
		lib_dir = IP6T_LIB_DIR;

#ifdef NO_SHARED_LIBS
	init_extensions();
#endif

	ret = do_command6(argc, argv, &table, &handle);
	if (ret)
		ret = ip6tc_commit(&handle);

	if (!ret)
		fprintf(stderr, "ip6tables: %s\n",
			ip6tc_strerror(errno));

	exit(!ret);
}

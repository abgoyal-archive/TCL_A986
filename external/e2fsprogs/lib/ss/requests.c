
#include <stdio.h>
#include "ss_internal.h"

#define	DECLARE(name) void name(int argc,const char * const *argv, \
				int sci_idx, void *infop)
	
void ss_self_identify(int argc __SS_ATTR((unused)),
		      const char * const *argv __SS_ATTR((unused)),
		      int sci_idx, void *infop __SS_ATTR((unused)))
{
     register ss_data *info = ss_info(sci_idx);
     printf("%s version %s\n", info->subsystem_name,
	    info->subsystem_version);
}

void ss_subsystem_name(int argc __SS_ATTR((unused)),
		       const char * const *argv __SS_ATTR((unused)),
		       int sci_idx, 
		       void *infop __SS_ATTR((unused)))
{
     printf("%s\n", ss_info(sci_idx)->subsystem_name);
}

void ss_subsystem_version(int argc __SS_ATTR((unused)),
			  const char * const *argv __SS_ATTR((unused)),
			  int sci_idx, 
			  void *infop __SS_ATTR((unused)))
{
     printf("%s\n", ss_info(sci_idx)->subsystem_version);
}

void ss_unimplemented(int argc __SS_ATTR((unused)),
		      const char * const *argv __SS_ATTR((unused)),
		      int sci_idx, void *infop __SS_ATTR((unused)))
{
     ss_perror(sci_idx, SS_ET_UNIMPLEMENTED, "");
}

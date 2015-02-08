


#include <cs/bfa_sm.h>


int
bfa_sm_to_state(struct bfa_sm_table_s *smt, bfa_sm_t sm)
{
	int             i = 0;

	while (smt[i].sm && smt[i].sm != sm)
		i++;
	return smt[i].state;
}





#include <linux/types.h>
#include <net/sctp/sctp.h>
#include <net/sctp/sm.h>

/* Initialize a block of memory as a command sequence. */
int sctp_init_cmd_seq(sctp_cmd_seq_t *seq)
{
	memset(seq, 0, sizeof(sctp_cmd_seq_t));
	return 1;		/* We always succeed.  */
}

void sctp_add_cmd_sf(sctp_cmd_seq_t *seq, sctp_verb_t verb, sctp_arg_t obj)
{
	BUG_ON(seq->next_free_slot >= SCTP_MAX_NUM_COMMANDS);

	seq->cmds[seq->next_free_slot].verb = verb;
	seq->cmds[seq->next_free_slot++].obj = obj;
}

sctp_cmd_t *sctp_next_cmd(sctp_cmd_seq_t *seq)
{
	sctp_cmd_t *retval = NULL;

	if (seq->next_cmd < seq->next_free_slot)
		retval = &seq->cmds[seq->next_cmd++];

	return retval;
}


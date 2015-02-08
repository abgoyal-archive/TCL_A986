
#include "includes.h"
#include "channel.h"
#include "buffer.h"
#include "circbuffer.h"
#include "dbutil.h"
#include "session.h"
#include "ssh.h"

/* We receive channel data - only used by the client chansession code*/
void recv_msg_channel_extended_data() {

	struct Channel *channel;
	unsigned int datatype;

	TRACE(("enter recv_msg_channel_extended_data"))

	channel = getchannel();

	if (channel->type != &clichansess) {
		TRACE(("leave recv_msg_channel_extended_data: chantype is wrong"))
		return; /* we just ignore it */
	}

	datatype = buf_getint(ses.payload);
	
	if (datatype != SSH_EXTENDED_DATA_STDERR) {
		TRACE(("leave recv_msg_channel_extended_data: wrong datatype: %d",
					datatype))
		return;	
	}

	common_recv_msg_channel_data(channel, channel->errfd, channel->extrabuf);

	TRACE(("leave recv_msg_channel_extended_data"))
}



#ifndef _TIPC_NAME_DISTR_H
#define _TIPC_NAME_DISTR_H

#include "name_table.h"

void tipc_named_publish(struct publication *publ);
void tipc_named_withdraw(struct publication *publ);
void tipc_named_node_up(unsigned long node);
void tipc_named_recv(struct sk_buff *buf);
void tipc_named_reinit(void);

#endif

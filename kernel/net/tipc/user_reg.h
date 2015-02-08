

#ifndef _TIPC_USER_REG_H
#define _TIPC_USER_REG_H

#include "port.h"

int tipc_reg_start(void);
void tipc_reg_stop(void);

int tipc_reg_add_port(struct user_port *up_ptr);
int tipc_reg_remove_port(struct user_port *up_ptr);

#endif

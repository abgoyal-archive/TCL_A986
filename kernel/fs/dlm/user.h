

#ifndef __USER_DOT_H__
#define __USER_DOT_H__

void dlm_user_add_ast(struct dlm_lkb *lkb, int type, int mode);
int dlm_user_init(void);
void dlm_user_exit(void);
int dlm_device_deregister(struct dlm_ls *ls);
int dlm_user_daemon_available(void);

#endif

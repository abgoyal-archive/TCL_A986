

#ifndef __ASTD_DOT_H__
#define __ASTD_DOT_H__

void dlm_add_ast(struct dlm_lkb *lkb, int type, int mode);
void dlm_del_ast(struct dlm_lkb *lkb);

void dlm_astd_wake(void);
int dlm_astd_start(void);
void dlm_astd_stop(void);
void dlm_astd_suspend(void);
void dlm_astd_resume(void);

#endif


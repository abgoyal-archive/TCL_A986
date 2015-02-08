
#ifndef _ASM_REBOOT_H
#define _ASM_REBOOT_H

extern void (*_machine_restart)(char *command);
extern void (*_machine_halt)(void);

#endif /* _ASM_REBOOT_H */

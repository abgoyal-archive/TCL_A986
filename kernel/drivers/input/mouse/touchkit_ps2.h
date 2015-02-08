

#ifndef _TOUCHKIT_PS2_H
#define _TOUCHKIT_PS2_H

#ifdef CONFIG_MOUSE_PS2_TOUCHKIT
int touchkit_ps2_detect(struct psmouse *psmouse, bool set_properties);
#else
static inline int touchkit_ps2_detect(struct psmouse *psmouse,
				      bool set_properties)
{
	return -ENOSYS;
}
#endif /* CONFIG_MOUSE_PS2_TOUCHKIT */

#endif

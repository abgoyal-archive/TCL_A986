
#include <linux/kernel.h>

extern unsigned long lvl14_save[5];
static unsigned long *linux_lvl14 = NULL;
static unsigned long obp_lvl14[4];
 
void install_linux_ticker(void)
{

	if (!linux_lvl14)
		return;
	linux_lvl14[0] =  lvl14_save[0];
	linux_lvl14[1] =  lvl14_save[1];
	linux_lvl14[2] =  lvl14_save[2];
	linux_lvl14[3] =  lvl14_save[3];
}

void install_obp_ticker(void)
{

	if (!linux_lvl14)
		return;
	linux_lvl14[0] =  obp_lvl14[0];
	linux_lvl14[1] =  obp_lvl14[1];
	linux_lvl14[2] =  obp_lvl14[2];
	linux_lvl14[3] =  obp_lvl14[3]; 
}

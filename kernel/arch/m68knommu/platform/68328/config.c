
/***************************************************************************/


/***************************************************************************/

#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <asm/machdep.h>
#include <asm/MC68328.h>

/***************************************************************************/

void m68328_timer_gettod(int *year, int *mon, int *day, int *hour, int *min, int *sec);

/***************************************************************************/

void m68328_reset (void)
{
  local_irq_disable();
  asm volatile ("moveal #0x10c00000, %a0;\n\t"
		"moveb #0, 0xFFFFF300;\n\t"
		"moveal 0(%a0), %sp;\n\t"
		"moveal 4(%a0), %a0;\n\t"
		"jmp (%a0);");
}

/***************************************************************************/

void config_BSP(char *command, int len)
{
  printk(KERN_INFO "\n68328 support D. Jeff Dionne <jeff@uclinux.org>\n");
  printk(KERN_INFO "68328 support Kenneth Albanowski <kjahds@kjshds.com>\n");
  printk(KERN_INFO "68328/Pilot support Bernhard Kuhn <kuhn@lpr.e-technik.tu-muenchen.de>\n");

  mach_gettod = m68328_timer_gettod;
  mach_reset = m68328_reset;
}

/***************************************************************************/

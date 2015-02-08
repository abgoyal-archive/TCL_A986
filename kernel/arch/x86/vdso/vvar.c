
#include <linux/kernel.h>
#include <linux/time.h>
#include <asm/vsyscall.h>
#include <asm/timex.h>
#include <asm/vgtod.h>

#define VEXTERN(x) typeof (__ ## x) *const vdso_ ## x = (void *)VMAGIC;
#include "vextern.h"

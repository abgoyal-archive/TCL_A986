
#ifndef __VICTOR_MPC30X_H
#define __VICTOR_MPC30X_H

#include <asm/vr41xx/irq.h>

#define VRC4173_PIN			1
#define MQ200_PIN			4

#define VRC4173_CASCADE_IRQ		GIU_IRQ(VRC4173_PIN)
#define MQ200_IRQ			GIU_IRQ(MQ200_PIN)

#endif /* __VICTOR_MPC30X_H */


#include "qemu-common.h"
#include "irq.h"

struct IRQState {
    qemu_irq_handler handler;
    void *opaque;
    int n;
};

void qemu_set_irq(qemu_irq irq, int level)
{
    if (!irq)
        return;

    irq->handler(irq->opaque, irq->n, level);
}

qemu_irq *qemu_allocate_irqs(qemu_irq_handler handler, void *opaque, int n)
{
    qemu_irq *s;
    struct IRQState *p;
    int i;

    s = (qemu_irq *)qemu_mallocz(sizeof(qemu_irq) * n);
    p = (struct IRQState *)qemu_mallocz(sizeof(struct IRQState) * n);
    for (i = 0; i < n; i++) {
        p->handler = handler;
        p->opaque = opaque;
        p->n = i;
        s[i] = p;
        p++;
    }
    return s;
}

void qemu_free_irqs(qemu_irq *s)
{
    qemu_free(s[0]);
    qemu_free(s);
}

static void qemu_notirq(void *opaque, int line, int level)
{
    struct IRQState *irq = opaque;

    irq->handler(irq->opaque, irq->n, !level);
}

qemu_irq qemu_irq_invert(qemu_irq irq)
{
    /* The default state for IRQs is low, so raise the output now.  */
    qemu_irq_raise(irq);
    return qemu_allocate_irqs(qemu_notirq, irq, 1)[0];
}

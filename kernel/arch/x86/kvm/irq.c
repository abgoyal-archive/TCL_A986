

#include <linux/module.h>
#include <linux/kvm_host.h>

#include "irq.h"
#include "i8254.h"
#include "x86.h"

int kvm_cpu_has_pending_timer(struct kvm_vcpu *vcpu)
{
	int ret;

	ret = pit_has_pending_timer(vcpu);
	ret |= apic_has_pending_timer(vcpu);

	return ret;
}
EXPORT_SYMBOL(kvm_cpu_has_pending_timer);

int kvm_cpu_has_interrupt(struct kvm_vcpu *v)
{
	struct kvm_pic *s;

	if (!irqchip_in_kernel(v->kvm))
		return v->arch.interrupt.pending;

	if (kvm_apic_has_interrupt(v) == -1) {	/* LAPIC */
		if (kvm_apic_accept_pic_intr(v)) {
			s = pic_irqchip(v->kvm);	/* PIC */
			return s->output;
		} else
			return 0;
	}
	return 1;
}
EXPORT_SYMBOL_GPL(kvm_cpu_has_interrupt);

int kvm_cpu_get_interrupt(struct kvm_vcpu *v)
{
	struct kvm_pic *s;
	int vector;

	if (!irqchip_in_kernel(v->kvm))
		return v->arch.interrupt.nr;

	vector = kvm_get_apic_interrupt(v);	/* APIC */
	if (vector == -1) {
		if (kvm_apic_accept_pic_intr(v)) {
			s = pic_irqchip(v->kvm);
			s->output = 0;		/* PIC */
			vector = kvm_pic_read_irq(v->kvm);
		}
	}
	return vector;
}
EXPORT_SYMBOL_GPL(kvm_cpu_get_interrupt);

void kvm_inject_pending_timer_irqs(struct kvm_vcpu *vcpu)
{
	kvm_inject_apic_timer_irqs(vcpu);
	kvm_inject_pit_timer_irqs(vcpu);
	/* TODO: PIT, RTC etc. */
}
EXPORT_SYMBOL_GPL(kvm_inject_pending_timer_irqs);

void __kvm_migrate_timers(struct kvm_vcpu *vcpu)
{
	__kvm_migrate_apic_timer(vcpu);
	__kvm_migrate_pit_timer(vcpu);
}

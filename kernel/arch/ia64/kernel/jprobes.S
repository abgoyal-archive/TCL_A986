
#include <asm/asmmacro.h>
#include <asm/break.h>

	/*
	 * void jprobe_break(void)
	 */
	.section .kprobes.text, "ax"
ENTRY(jprobe_break)
	break.m __IA64_BREAK_JPROBE
END(jprobe_break)

	/*
	 * void jprobe_inst_return(void)
	 */
GLOBAL_ENTRY(jprobe_inst_return)
	br.call.sptk.many b0=jprobe_break
END(jprobe_inst_return)

GLOBAL_ENTRY(invalidate_stacked_regs)
	movl r16=invalidate_restore_cfm
	;;
	mov b6=r16
	;;
	br.ret.sptk.many b6
	;;
invalidate_restore_cfm:
	mov r16=ar.rsc
	;;
	mov ar.rsc=r0
	;;
	loadrs
	;;
	mov ar.rsc=r16
	;;
	br.cond.sptk.many rp
END(invalidate_stacked_regs)

GLOBAL_ENTRY(flush_register_stack)
	// flush dirty regs to backing store (must be first in insn group)
	flushrs
	;;
	br.ret.sptk.many rp
END(flush_register_stack)


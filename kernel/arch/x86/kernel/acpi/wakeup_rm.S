
	.section ".rodata","a"
	.globl	wakeup_code_start, wakeup_code_end
wakeup_code_start:
	.incbin	"arch/x86/kernel/acpi/realmode/wakeup.bin"
wakeup_code_end:
	.size	wakeup_code_start, .-wakeup_code_start

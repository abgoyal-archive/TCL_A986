
	.text

###############################################################################
#
# kernel_thread_helper - trampoline for kernel_thread()
#
# On entry:
#   A2 = address of function to call
#   D2 = function argument
#
###############################################################################
	.globl	kernel_thread_helper
	.type	kernel_thread_helper,@function
kernel_thread_helper:
	mov	do_exit,d1
	mov	d1,(sp)
	mov	d1,mdr
	mov	d2,d0
	jmp	(a2)

	.size	kernel_thread_helper,.-kernel_thread_helper

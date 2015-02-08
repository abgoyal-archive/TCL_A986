

void
panic_handler(unsigned long panicPC, unsigned long panicSSR,
	      unsigned long panicEXPEVT)
{
	/* Never return from the panic handler */
	for (;;) ;
}

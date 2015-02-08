include/generated/machtypes.h: $(src)/gen-mach-types $(src)/mach-types
	@echo '  Generating $@'
	$(Q)mkdir -p $(dir $@)
	$(Q)LC_ALL=C $(AWK) -f $^ > $@ || { rm -f $@; /bin/false; }

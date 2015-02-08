include/generated/mach-types.h: $(src)/gen-mach-types $(src)/mach-types
	@echo '  Generating $@'
	@mkdir -p $(dir $@)
	$(Q)$(AWK) -f $^ > $@ || { rm -f $@; /bin/false; }

#!/usr/bin/perl -w
require 5;	# at least perl 5
use strict;
use File::Find;

my $nm = ($ENV{'NM'} || "nm") . " -p";
my $objdump = ($ENV{'OBJDUMP'} || "objdump") . " -s -j .comment";
my $srctree = "";
my $objtree = "";
$srctree = "$ENV{'srctree'}/" if (exists($ENV{'srctree'}));
$objtree = "$ENV{'objtree'}/" if (exists($ENV{'objtree'}));

if ($#ARGV != -1) {
	print STDERR "usage: $0 takes no parameters\n";
	die("giving up\n");
}

my %nmdata = ();	# nm data for each object
my %def = ();		# all definitions for each name
my %ksymtab = ();	# names that appear in __ksymtab_
my %ref = ();		# $ref{$name} exists if there is a true external reference to $name
my %export = ();	# $export{$name} exists if there is an EXPORT_... of $name

&find(\&linux_objects, '.');	# find the objects and do_nm on them
&list_multiply_defined();
&resolve_external_references();
&list_extra_externals();

exit(0);

sub linux_objects
{
	# Select objects, ignoring objects which are only created by
	# merging other objects.  Also ignore all of modules, scripts
	# and compressed.  Most conglomerate objects are handled by do_nm,
	# this list only contains the special cases.  These include objects
	# that are linked from just one other object and objects for which
	# there is really no permanent source file.
	my $basename = $_;
	$_ = $File::Find::name;
	s:^\./::;
	if (/.*\.o$/ &&
		! (
		m:/built-in.o$:
		|| m:arch/x86/kernel/vsyscall-syms.o$:
		|| m:arch/ia64/ia32/ia32.o$:
		|| m:arch/ia64/kernel/gate-syms.o$:
		|| m:arch/ia64/lib/__divdi3.o$:
		|| m:arch/ia64/lib/__divsi3.o$:
		|| m:arch/ia64/lib/__moddi3.o$:
		|| m:arch/ia64/lib/__modsi3.o$:
		|| m:arch/ia64/lib/__udivdi3.o$:
		|| m:arch/ia64/lib/__udivsi3.o$:
		|| m:arch/ia64/lib/__umoddi3.o$:
		|| m:arch/ia64/lib/__umodsi3.o$:
		|| m:arch/ia64/scripts/check_gas_for_hint.o$:
		|| m:arch/ia64/sn/kernel/xp.o$:
		|| m:boot/bbootsect.o$:
		|| m:boot/bsetup.o$:
		|| m:/bootsect.o$:
		|| m:/boot/setup.o$:
		|| m:/compressed/:
		|| m:drivers/cdrom/driver.o$:
		|| m:drivers/char/drm/tdfx_drv.o$:
		|| m:drivers/ide/ide-detect.o$:
		|| m:drivers/ide/pci/idedriver-pci.o$:
		|| m:drivers/media/media.o$:
		|| m:drivers/scsi/sd_mod.o$:
		|| m:drivers/video/video.o$:
		|| m:fs/devpts/devpts.o$:
		|| m:fs/exportfs/exportfs.o$:
		|| m:fs/hugetlbfs/hugetlbfs.o$:
		|| m:fs/msdos/msdos.o$:
		|| m:fs/nls/nls.o$:
		|| m:fs/ramfs/ramfs.o$:
		|| m:fs/romfs/romfs.o$:
		|| m:fs/vfat/vfat.o$:
		|| m:init/mounts.o$:
		|| m:^modules/:
		|| m:net/netlink/netlink.o$:
		|| m:net/sched/sched.o$:
		|| m:/piggy.o$:
		|| m:^scripts/:
		|| m:sound/.*/snd-:
		|| m:^.*/\.tmp_:
		|| m:^\.tmp_:
		|| m:/vmlinux-obj.o$:
		)
	) {
		do_nm($basename, $_);
	}
	$_ = $basename;		# File::Find expects $_ untouched (undocumented)
}

sub do_nm
{
	my ($basename, $fullname) = @_;
	my ($source, $type, $name);
	if (! -e $basename) {
		printf STDERR "$basename does not exist\n";
		return;
	}
	if ($fullname !~ /\.o$/) {
		printf STDERR "$fullname is not an object file\n";
		return;
	}
	($source = $fullname) =~ s/\.o$//;
	if (-e "$objtree$source.c" || -e "$objtree$source.S") {
		$source = "$objtree$source";
	} else {
		$source = "$srctree$source";
	}
	if (! -e "$source.c" && ! -e "$source.S") {
		# No obvious source, exclude the object if it is conglomerate
	        open(my $objdumpdata, "$objdump $basename|")
		    or die "$objdump $fullname failed $!\n";

		my $comment;
		while (<$objdumpdata>) {
			chomp();
			if (/^In archive/) {
				# Archives are always conglomerate
				$comment = "GCC:GCC:";
				last;
			}
			next if (! /^[ 0-9a-f]{5,} /);
			$comment .= substr($_, 43);
		}
		close($objdumpdata);

		if (!defined($comment) || $comment !~ /GCC\:.*GCC\:/m) {
			printf STDERR "No source file found for $fullname\n";
		}
		return;
	}
	open (my $nmdata, "$nm $basename|")
	    or die "$nm $fullname failed $!\n";

	my @nmdata;
	while (<$nmdata>) {
		chop;
		($type, $name) = (split(/ +/, $_, 3))[1..2];
		# Expected types
		# A absolute symbol
		# B weak external reference to data that has been resolved
		# C global variable, uninitialised
		# D global variable, initialised
		# G global variable, initialised, small data section
		# R global array, initialised
		# S global variable, uninitialised, small bss
		# T global label/procedure
		# U external reference
		# W weak external reference to text that has been resolved
		# a assembler equate
		# b static variable, uninitialised
		# d static variable, initialised
		# g static variable, initialised, small data section
		# r static array, initialised
		# s static variable, uninitialised, small bss
		# t static label/procedures
		# w weak external reference to text that has not been resolved
		# ? undefined type, used a lot by modules
		if ($type !~ /^[ABCDGRSTUWabdgrstw?]$/) {
			printf STDERR "nm output for $fullname contains unknown type '$_'\n";
		}
		elsif ($name =~ /\./) {
			# name with '.' is local static
		}
		else {
			$type = 'R' if ($type eq '?');	# binutils replaced ? with R at one point
			# binutils keeps changing the type for exported symbols, force it to R
			$type = 'R' if ($name =~ /^__ksymtab/ || $name =~ /^__kstrtab/);
			$name =~ s/_R[a-f0-9]{8}$//;	# module versions adds this
			if ($type =~ /[ABCDGRSTW]/ &&
				$name ne 'init_module' &&
				$name ne 'cleanup_module' &&
				$name ne 'Using_Versions' &&
				$name !~ /^Version_[0-9]+$/ &&
				$name !~ /^__parm_/ &&
				$name !~ /^__kstrtab/ &&
				$name !~ /^__ksymtab/ &&
				$name !~ /^__kcrctab_/ &&
				$name !~ /^__exitcall_/ &&
				$name !~ /^__initcall_/ &&
				$name !~ /^__kdb_initcall_/ &&
				$name !~ /^__kdb_exitcall_/ &&
				$name !~ /^__module_/ &&
				$name !~ /^__mod_/ &&
				$name !~ /^__crc_/ &&
				$name ne '__this_module' &&
				$name ne 'kernel_version') {
				if (!exists($def{$name})) {
					$def{$name} = [];
				}
				push(@{$def{$name}}, $fullname);
			}
			push(@nmdata, "$type $name");
			if ($name =~ /^__ksymtab_/) {
				$name = substr($name, 10);
				if (!exists($ksymtab{$name})) {
					$ksymtab{$name} = [];
				}
				push(@{$ksymtab{$name}}, $fullname);
			}
		}
	}
	close($nmdata);

	if ($#nmdata < 0) {
		if (
			$fullname ne "lib/brlock.o"
			&& $fullname ne "lib/dec_and_lock.o"
			&& $fullname ne "fs/xfs/xfs_macros.o"
			&& $fullname ne "drivers/ide/ide-probe-mini.o"
			&& $fullname ne "usr/initramfs_data.o"
			&& $fullname ne "drivers/acpi/executer/exdump.o"
			&& $fullname ne "drivers/acpi/resources/rsdump.o"
			&& $fullname ne "drivers/acpi/namespace/nsdumpdv.o"
			&& $fullname ne "drivers/acpi/namespace/nsdump.o"
			&& $fullname ne "arch/ia64/sn/kernel/sn2/io.o"
			&& $fullname ne "arch/ia64/kernel/gate-data.o"
			&& $fullname ne "drivers/ieee1394/oui.o"
			&& $fullname ne "security/capability.o"
			&& $fullname ne "sound/core/wrappers.o"
			&& $fullname ne "fs/ntfs/sysctl.o"
			&& $fullname ne "fs/jfs/jfs_debug.o"
		) {
			printf "No nm data for $fullname\n";
		}
		return;
	}
	$nmdata{$fullname} = \@nmdata;
}

sub drop_def
{
	my ($object, $name) = @_;
	my $nmdata = $nmdata{$object};
	my ($i, $j);
	for ($i = 0; $i <= $#{$nmdata}; ++$i) {
		if ($name eq (split(' ', $nmdata->[$i], 2))[1]) {
			splice(@{$nmdata{$object}}, $i, 1);
			my $def = $def{$name};
			for ($j = 0; $j < $#{$def{$name}}; ++$j) {
				if ($def{$name}[$j] eq $object) {
					splice(@{$def{$name}}, $j, 1);
				}
			}
			last;
		}
	}
}

sub list_multiply_defined
{
	foreach my $name (keys(%def)) {
		if ($#{$def{$name}} > 0) {
			# Special case for cond_syscall
			if ($#{$def{$name}} == 1 && $name =~ /^sys_/ &&
			    ($def{$name}[0] eq "kernel/sys.o" ||
			     $def{$name}[1] eq "kernel/sys.o")) {
				&drop_def("kernel/sys.o", $name);
				next;
			}
			# Special case for i386 entry code
			if ($#{$def{$name}} == 1 && $name =~ /^__kernel_/ &&
			    $def{$name}[0] eq "arch/x86/kernel/vsyscall-int80_32.o" &&
			    $def{$name}[1] eq "arch/x86/kernel/vsyscall-sysenter_32.o") {
				&drop_def("arch/x86/kernel/vsyscall-sysenter_32.o", $name);
				next;
			}

			printf "$name is multiply defined in :-\n";
			foreach my $module (@{$def{$name}}) {
				printf "\t$module\n";
			}
		}
	}
}

sub resolve_external_references
{
	my ($kstrtab, $ksymtab, $export);

	printf "\n";
	foreach my $object (keys(%nmdata)) {
		my $nmdata = $nmdata{$object};
		for (my $i = 0; $i <= $#{$nmdata}; ++$i) {
			my ($type, $name) = split(' ', $nmdata->[$i], 2);
			if ($type eq "U" || $type eq "w") {
				if (exists($def{$name}) || exists($ksymtab{$name})) {
					# add the owning object to the nmdata
					$nmdata->[$i] = "$type $name $object";
					# only count as a reference if it is not EXPORT_...
					$kstrtab = "R __kstrtab_$name";
					$ksymtab = "R __ksymtab_$name";
					$export = 0;
					for (my $j = 0; $j <= $#{$nmdata}; ++$j) {
						if ($nmdata->[$j] eq $kstrtab ||
						    $nmdata->[$j] eq $ksymtab) {
							$export = 1;
							last;
						}
					}
					if ($export) {
						$export{$name} = "";
					}
					else {
						$ref{$name} = ""
					}
				}
				elsif (    $name ne "mod_use_count_"
					&& $name ne "__initramfs_end"
					&& $name ne "__initramfs_start"
					&& $name ne "_einittext"
					&& $name ne "_sinittext"
					&& $name ne "kallsyms_names"
					&& $name ne "kallsyms_num_syms"
					&& $name ne "kallsyms_addresses"
					&& $name ne "__this_module"
					&& $name ne "_etext"
					&& $name ne "_edata"
					&& $name ne "_end"
					&& $name ne "__bss_start"
					&& $name ne "_text"
					&& $name ne "_stext"
					&& $name ne "__gp"
					&& $name ne "ia64_unw_start"
					&& $name ne "ia64_unw_end"
					&& $name ne "__init_begin"
					&& $name ne "__init_end"
					&& $name ne "__bss_stop"
					&& $name ne "__nosave_begin"
					&& $name ne "__nosave_end"
					&& $name ne "pg0"
					&& $name ne "__module_text_address"
					&& $name !~ /^__sched_text_/
					&& $name !~ /^__start_/
					&& $name !~ /^__end_/
					&& $name !~ /^__stop_/
					&& $name !~ /^__scheduling_functions_.*_here/
					&& $name !~ /^__.*initcall_/
					&& $name !~ /^__.*per_cpu_start/
					&& $name !~ /^__.*per_cpu_end/
					&& $name !~ /^__alt_instructions/
					&& $name !~ /^__setup_/
					&& $name !~ /^jiffies/
					&& $name !~ /^__mod_timer/
					&& $name !~ /^__mod_page_state/
					&& $name !~ /^init_module/
					&& $name !~ /^cleanup_module/
				) {
					printf "Cannot resolve ";
					printf "weak " if ($type eq "w");
					printf "reference to $name from $object\n";
				}
			}
		}
	}
}

sub list_extra_externals
{
	my %noref = ();

	foreach my $name (keys(%def)) {
		if (! exists($ref{$name})) {
			my @module = @{$def{$name}};
			foreach my $module (@module) {
				if (! exists($noref{$module})) {
					$noref{$module} = [];
				}
				push(@{$noref{$module}}, $name);
			}
		}
	}
	if (%noref) {
		printf "\nExternally defined symbols with no external references\n";
		foreach my $module (sort(keys(%noref))) {
			printf "  $module\n";
			foreach (sort(@{$noref{$module}})) {
			    my $export;
			    if (exists($export{$_})) {
				$export = " (export only)";
			    } else {
				$export = "";
			    }
			    printf "    $_$export\n";
			}
		}
	}
}

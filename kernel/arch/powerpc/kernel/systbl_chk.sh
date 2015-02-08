#!/bin/sh
awk	'BEGIN { num = -1; }	# Ignore the beginning of the file
	/^#/ { next; }
	/^[ \t]*$/ { next; }
	/^START_TABLE/ { num = 0; next; }
	/^END_TABLE/ {
		if (num != $2) {
			printf "__NR_syscalls (%s) is not one more than the last syscall (%s)\n",
				$2, num - 1;
			exit(1);
		}
		num = -1;	# Ignore the rest of the file
	}
	{
		if (num == -1) next;
		if (($1 != -1) && ($1 != num)) {
			printf "Syscall %s out of order (expected %s)\n",
				$1, num;
			exit(1);
		};
		num++;
	}' "$1"

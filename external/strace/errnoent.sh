#!/bin/sh
awk '
/^#define[ 	]+E[A-Z0-9_]+[ 	]+[0-9]+/ {
	errno[$3] = $2
	if ($3 > max)
		max = $3
}
END {
	for (i = 0; i <= max; i++) {
		if (!errno[i])
			errno[i] = "ERRNO_" i
		printf "\t\"%s\", /* %d */\n", errno[i], i
	}
}
' $*

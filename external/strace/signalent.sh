#!/bin/sh
cat $* |
	sed -n -e 's/\/\*.*\*\// /' -e 's/^#[ 	]*define[ 	][ 	]*SIG\([^_ 	]*\)[ 	][ 	]*\([0-9][0-9]*\)[ 	]*$/\1 \2/p' |
	sort +1n |
	awk '
	BEGIN {
		tabs = "\t\t\t\t\t\t\t\t"
		signal = -1;
	}
	$2 <= 256 {
		if (signal == $2)
			next
		while (++signal < $2) {
			n = "\"SIG_" signal "\""
			s = "\t" n ","
			s = s substr(tabs, 1, 16/8 - int((length(n) + 1)/8))
			s = s "/* " signal " */"
			print s
		}
		if (signal == $2)
			n = "\"SIG" $1 "\""
		n = "\"SIG" $1 "\""
		s = "\t" n ","
		s = s substr(tabs, 1, 16/8 - int((length(n) + 1)/8))
		s = s "/* " signal " */"
		print s
	}
	'

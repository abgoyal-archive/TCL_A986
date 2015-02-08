#!/usr/bin/env python
import sys
import struct

i = 0
while True:
	buf = sys.stdin.read(4)

	if len(buf) == 0:
		break
	elif len(buf) != 4:
		sys.stdout.write("\n")
		sys.stderr.write("Error: read {0} not 4 bytes\n".format(len(buf)))
		sys.exit(1)

	if i > 0:
		sys.stdout.write(" ")
	sys.stdout.write("{0:x}={1}".format(i, struct.unpack("<I", buf)[0]))
	i += 1

sys.stdout.write("\n")

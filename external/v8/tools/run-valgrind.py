#!/usr/bin/python
import subprocess
import sys
import re

VALGRIND_ARGUMENTS = [
  'valgrind',
  '--error-exitcode=1',
  '--leak-check=full',
  '--smc-check=all'
]

# Compute the command line.
command = VALGRIND_ARGUMENTS + sys.argv[1:]

# Run valgrind.
process = subprocess.Popen(command, stderr=subprocess.PIPE)
code = process.wait();
errors = process.stderr.readlines();

# If valgrind produced an error, we report that to the user.
if code != 0:
  sys.stderr.writelines(errors)
  sys.exit(code)

# Look through the leak details and make sure that we don't
# have any definitely, indirectly, and possibly lost bytes.
LEAK_RE = r"(?:definitely|indirectly|possibly) lost: "
LEAK_LINE_MATCHER = re.compile(LEAK_RE)
LEAK_OKAY_MATCHER = re.compile(r"lost: 0 bytes in 0 blocks")
leaks = []
for line in errors:
  if LEAK_LINE_MATCHER.search(line):
    leaks.append(line)
    if not LEAK_OKAY_MATCHER.search(line):
      sys.stderr.writelines(errors)
      sys.exit(1)

# Make sure we found between 2 and 3 leak lines.
if len(leaks) < 2 or len(leaks) > 3:
  sys.stderr.writelines(errors)
  sys.stderr.write('\n\n#### Malformed valgrind output.\n#### Exiting.\n')
  sys.exit(1)

# No leaks found.
sys.exit(0)

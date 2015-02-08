#!/usr/bin/python
import os
import subprocess
import sys


def SplitArgsIntoSections(args):
    sections = []
    while len(args) > 0:
        if not '--' in args:
            # If there is no '--' left, everything remaining is an entire section.
            dashes = len(args)
        else:
            dashes = args.index('--')

        sections.append(args[:dashes])

        # Next time through the loop, look at everything after this '--'.
        if dashes + 1 == len(args):
            # If the '--' is at the end of the list, we won't come back through the
            # loop again. Add an empty section now corresponding to the nothingness
            # following the final '--'.
            args = []
            sections.append(args)
        else:
            args = args[dashes + 1:]

    return sections


def main(args):
    sections = SplitArgsIntoSections(args[1:])
    assert len(sections) == 2
    (outputs, inputs) = sections

    assert len(outputs) == 1
    output = outputs[0]

    assert len(inputs) == 2
    maketokenizer = inputs[0]
    flexInput = inputs[1]

    # Do it. check_call is new in 2.5, so simulate its behavior with call and
    # assert.
    outfile = open(output, 'wb')
    p1 = subprocess.Popen(['flex', '-t', flexInput], stdout=subprocess.PIPE)
    p2 = subprocess.Popen(['perl', maketokenizer], stdin=p1.stdout, stdout=outfile)

    r1 = p1.wait()
    r2 = p2.wait()
    assert r1 == 0
    assert r2 == 0

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))

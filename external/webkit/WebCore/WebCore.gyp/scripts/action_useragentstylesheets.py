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

    assert len(outputs) == 2
    outputH = outputs[0]
    outputCpp = outputs[1]

    makeCssFileArrays = inputs[0]
    styleSheets = inputs[1:]

    # Build up the command.
    command = ['perl', makeCssFileArrays, outputH, outputCpp]
    command.extend(styleSheets)

    # Do it. check_call is new in 2.5, so simulate its behavior with call and
    # assert.
    returnCode = subprocess.call(command)
    assert returnCode == 0

    return returnCode


if __name__ == '__main__':
    sys.exit(main(sys.argv))

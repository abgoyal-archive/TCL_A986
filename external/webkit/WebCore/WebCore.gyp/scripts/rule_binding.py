#!/usr/bin/python
import errno
import os
import shlex
import shutil
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
    assert len(sections) == 3, sections
    (base, inputs, options) = sections

    assert len(base) == 3, base
    (input, cppdir, hdir) = base

    assert len(inputs) > 1, inputs
    generateBindings = inputs[0]
    perlModules = inputs[1:]

    includeDirs = []
    for perlModule in perlModules:
        includeDir = os.path.dirname(perlModule)
        if not includeDir in includeDirs:
            includeDirs.append(includeDir)

    # The defines come in as one flat string. Split it up into distinct arguments.
    if '--defines' in options:
        definesIndex = options.index('--defines')
        if definesIndex + 1 < len(options):
            splitOptions = shlex.split(options[definesIndex + 1])
            if splitOptions:
                options[definesIndex + 1] = ' '.join(splitOptions)

    # Build up the command.
    command = ['perl', '-w']
    for includeDir in includeDirs:
        command.extend(['-I', includeDir])
    command.append(generateBindings)
    command.extend(options)
    command.extend(['--outputDir', cppdir, input])

    # Do it. check_call is new in 2.5, so simulate its behavior with call and
    # assert.
    returnCode = subprocess.call(command)
    assert returnCode == 0

    # Both the .cpp and .h were generated in cppdir, but if hdir is different,
    # the .h needs to move. Copy it instead of using os.rename for maximum
    # portability in all cases.
    if cppdir != hdir:
        inputBasename = os.path.basename(input)
        (root, ext) = os.path.splitext(inputBasename)
        hname = 'V8%s.h' % root
        hsrc = os.path.join(cppdir, hname)
        hdst = os.path.join(hdir, hname)
        shutil.copyfile(hsrc, hdst)
        os.unlink(hsrc)

    return returnCode


if __name__ == '__main__':
    sys.exit(main(sys.argv))

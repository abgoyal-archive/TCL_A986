#!/usr/bin/python
import os
import posixpath
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
    assert len(sections) == 2 or len(sections) == 3
    (outputs, inputs) = sections[:2]
    if len(sections) == 3:
        options = sections[2]
    else:
        options = []

    # Make all output pathnames absolute so that they can be accessed after
    # changing directory.
    for index in xrange(0, len(outputs)):
        outputs[index] = os.path.abspath(outputs[index])

    outputDir = os.path.dirname(outputs[0])

    # Look at the inputs and figure out which ones are make_names.pl, tags, and
    # attributes. There can be at most one of each, and those are the only
    # input types supported. make_names.pl is required and at least one of tags
    # and attributes is required.
    makeNamesInput = None
    tagInput = None
    attrInput = None
    for input in inputs:
        # Make input pathnames absolute so they can be accessed after changing
        # directory. On Windows, convert \ to / for inputs to the perl script to
        # work around the intermix of activepython + cygwin perl.
        inputAbs = os.path.abspath(input)
        inputAbsPosix = inputAbs.replace(os.path.sep, posixpath.sep)
        inputBasename = os.path.basename(input)
        if inputBasename == 'make_names.pl':
            assert makeNamesInput == None
            makeNamesInput = inputAbs
        elif inputBasename.endswith('TagNames.in') \
             or inputBasename.endswith('tags.in'):
            assert tagInput == None
            tagInput = inputAbsPosix
        elif inputBasename.endswith('AttributeNames.in') \
             or inputBasename.endswith('attrs.in'):
            assert attrInput == None
            attrInput = inputAbsPosix
        else:
            assert False

    assert makeNamesInput != None
    assert tagInput != None or attrInput != None

    # scriptsPath is a Perl include directory, located relative to
    # makeNamesInput.
    scriptsPath = os.path.normpath(
        os.path.join(os.path.dirname(makeNamesInput), os.pardir, 'bindings', 'scripts'))

    # Change to the output directory because make_names.pl puts output in its
    # working directory.
    os.chdir(outputDir)

    # Build up the command.
    command = ['perl', '-I', scriptsPath, makeNamesInput]
    if tagInput != None:
        command.extend(['--tags', tagInput])
    if attrInput != None:
        command.extend(['--attrs', attrInput])
    command.extend(options)

    # Do it. check_call is new in 2.5, so simulate its behavior with call and
    # assert.
    returnCode = subprocess.call(command)
    assert returnCode == 0

    # Go through the outputs. Any output that belongs in a different directory
    # is moved. Do a copy and delete instead of rename for maximum portability.
    # Note that all paths used in this section are still absolute.
    for output in outputs:
        thisOutputDir = os.path.dirname(output)
        if thisOutputDir != outputDir:
            outputBasename = os.path.basename(output)
            src = os.path.join(outputDir, outputBasename)
            dst = os.path.join(thisOutputDir, outputBasename)
            shutil.copyfile(src, dst)
            os.unlink(src)

    return returnCode


if __name__ == '__main__':
    sys.exit(main(sys.argv))

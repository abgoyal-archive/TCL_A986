#!/usr/bin/python
import posixpath
import shutil
import subprocess
import sys

assert len(sys.argv) == 3

inputFile = sys.argv[1]
outputDir = sys.argv[2]

gperfCommands = {
    'DocTypeStrings.gperf': [
        '-CEot', '-L', 'ANSI-C', '-k*', '-N', 'findDoctypeEntry',
        '-F', ',PubIDInfo::eAlmostStandards,PubIDInfo::eAlmostStandards'
    ],
    'HTMLEntityNames.gperf': [
        '-a', '-L', 'ANSI-C', '-C', '-G', '-c', '-o', '-t', '-k*',
        '-N', 'findEntity', '-D', '-s', '2'
    ],
    'ColorData.gperf': [
        '-CDEot', '-L', 'ANSI-C', '-k*', '-N', 'findColor', '-D', '-s', '2'
    ],
}

inputName = posixpath.basename(inputFile)
assert inputName in gperfCommands

(inputRoot, inputExt) = posixpath.splitext(inputName)
outputCpp = posixpath.join(outputDir, inputRoot + '.cpp')

#command = ['gperf', '--output-file', outputCpp]
command = ['gperf']
command.extend(gperfCommands[inputName])
command.append(inputFile)

ofile = open(outputCpp, 'w')

# Do it. check_call is new in 2.5, so simulate its behavior with call and
# assert.
returnCode = subprocess.call(command, stdout=ofile.fileno())
assert returnCode == 0

outputC = posixpath.join(outputDir, inputRoot + '.c')
shutil.copyfile(outputCpp, outputC)

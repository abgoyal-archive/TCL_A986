#!/usr/bin/python
import errno
import os
import os.path
import subprocess
import sys

assert len(sys.argv) == 3

inputFile = sys.argv[1]
outputDir = sys.argv[2]

inputName = os.path.basename(inputFile)
assert inputName == 'CSSGrammar.y' or inputName == 'XPathGrammar.y'
prefix = {'CSSGrammar.y': 'cssyy', 'XPathGrammar.y': 'xpathyy'}[inputName]

(inputRoot, inputExt) = os.path.splitext(inputName)

# The generated .h will be in a different location depending on the bison
# version.
outputHTries = [
    os.path.join(outputDir, inputRoot + '.cpp.h'),
    os.path.join(outputDir, inputRoot + '.hpp'),
]

for outputHTry in outputHTries:
    try:
        os.unlink(outputHTry)
    except OSError, e:
        if e.errno != errno.ENOENT:
            raise

outputCpp = os.path.join(outputDir, inputRoot + '.cpp')

returnCode = subprocess.call(['bison', '-d', '-p', prefix, inputFile, '-o', outputCpp])
assert returnCode == 0

# Find the name that bison used for the generated header file.
outputHTmp = None
for outputHTry in outputHTries:
    try:
        os.stat(outputHTry)
        outputHTmp = outputHTry
        break
    except OSError, e:
        if e.errno != errno.ENOENT:
            raise

assert outputHTmp != None

# Read the header file in under the generated name and remove it.
outputHFile = open(outputHTmp)
outputHContents = outputHFile.read()
outputHFile.close()
os.unlink(outputHTmp)

# Rewrite the generated header with #include guards.
outputH = os.path.join(outputDir, inputRoot + '.h')

outputHFile = open(outputH, 'w')
print >>outputHFile, '#ifndef %sH' % inputRoot
print >>outputHFile, '#define %sH' % inputRoot
print >>outputHFile, outputHContents
print >>outputHFile, '#endif'
outputHFile.close()

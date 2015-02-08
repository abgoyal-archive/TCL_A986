#!/usr/bin/env python
import sys, os, string
import commands
import glob
from subprocess import *

script_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, "..", "build")))

from build_utils import *

wxwk_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
wxwebkit_dir = os.path.abspath(os.path.join(wxwk_root, "WebKitBuild", get_config(wxwk_root) + git_branch_name()))

# Find InnoSetup executable
def getInnoSetupPath():
    name = "ISCC.exe"
    retval = ""
    dirs = os.environ["PATH"].split(":")
    # Add the default file path
    dirs.append("C:\\Program Files\\Inno Setup 5")
    dirs.append("C:\\Program Files (x86)\\Inno Setup 5")
                    
    if os.environ.has_key("INNO5"):
        retval = os.environ["INNO5"]
    
    if retval == "":
        for dir in dirs:
            filepath = os.path.join(dir, name)
            if os.path.isfile(filepath):
                retval = filepath
            
    return retval

if __name__ == "__main__":
    innoSetup = getInnoSetupPath()
    os.chdir(sys.path[0])

    svnrevision = svn_revision()

    if not os.path.exists(innoSetup):
        print "ERROR: Cannot find InnoSetup."
        #sys.exit(1)
        
    if not os.path.exists(wxwebkit_dir):
        print "ERROR: Build dir %s doesn't exist." % wxwebkit_dir
        sys.exit(1)

    fileList = """
CopyMode: alwaysoverwrite; Source: *.pyd;        DestDir: "{app}"
CopyMode: alwaysoverwrite; Source: *.py;        DestDir: "{app}"
"""
    
    dlls = glob.glob(os.path.join(wxwebkit_dir, "*.dll"))
    for dll in dlls:
        if dll.find("wxbase") == -1 and dll.find("wxmsw") == -1:
            fileList += """CopyMode: alwaysoverwrite; Source: %s;        DestDir: "{app}" \n""" % dll

    installerTemplate = open("wxWebKitInstaller.iss.in", "r").read()

    installerTemplate = installerTemplate.replace("<<VERSION>>", svnrevision)
    installerTemplate = installerTemplate.replace("<<ROOTDIR>>", wxwebkit_dir )
    installerTemplate = installerTemplate.replace("<<PYTHONVER>>", sys.version[0:3] )
    installerTemplate = installerTemplate.replace("<<FILES>>", fileList )

    outputFile = open("wxWebKitInstaller.iss", "w")
    outputFile.write(installerTemplate)
    outputFile.close()

    success = os.system('"%s" wxWebKitInstaller.iss' % innoSetup)
    sys.exit(success)

#!/usr/bin/env python
import commands
import distutils.sysconfig
import glob
import optparse
import os
import shutil
import string
import sys
import tempfile

script_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.append(os.path.abspath(os.path.join(script_dir, "..", "build")))

from build_utils import *

import wx

wxwk_root = os.path.abspath(os.path.join(script_dir, "..", "..", ".."))
wxwebkit_dir = os.path.abspath(os.path.join(wxwk_root, "WebKitBuild", get_config(wxwk_root) + git_branch_name()))

wx_version = wx.__version__[:5]
py_version = sys.version[:3]

wxwk_version = svn_revision()

platform = "osx"
    
pkgname = "wxWebKit-%s-wx%s-py%s" % (platform, wx_version[:3], py_version)

tempdir = "/tmp/%s" % (pkgname)

if os.path.exists(tempdir):
    shutil.rmtree(tempdir)
    os.makedirs(tempdir)

installroot = os.path.join(tempdir, "install-root")
installapps = os.path.join(tempdir, "install-apps")

sp_root = distutils.sysconfig.get_python_lib()
wx_root = sp_root
if sys.platform.startswith("darwin"):
    wx_root = "/usr/local/lib/wxPython-unicode-%s" % wx.__version__
    sp_root = "%s/lib/python%s/site-packages" % (wx_root, py_version)
sitepackages = "%s/wx-%s-mac-unicode/wx" % (sp_root, wx_version[:3])
prefix = sitepackages

def mac_update_dependencies(dylib, prefix):
    """
    Copies any non-system dependencies into the bundle, and
    updates the install name path to the new path in the bundle.
    """
    global wx_root
    system_prefixes = ["/usr/lib", "/System/Library", wx_root]

    output = commands.getoutput("otool -L %s" % dylib).strip()
    for line in output.split("\n"):
        filename = line.split("(")[0].strip()
        if os.path.exists(filename):
            print "checking dll %s" % filename
            copy = True
            for sys_prefix in system_prefixes:
                if filename.startswith(sys_prefix):
                    copy = False
            
            if copy:
                copydir = os.path.dirname(dylib)
                
                filedir, basename = os.path.split(filename)
                dest_filename = os.path.join(prefix, basename)
                copyname = os.path.join(copydir, basename)
                if not os.path.exists(copyname):
                    shutil.copy(filename, copydir)
                    os.system("install_name_tool -id %s %s" % (dest_filename, copyname))
                
                os.system("install_name_tool -change %s %s %s" % (filename, dest_filename, dylib))

def exitIfError(cmd):
    print cmd
    retval = os.system(cmd)
    if retval != 0:
        if os.path.exists(tempdir):
            shutil.rmtree(tempdir)
        sys.exit(1)

wxroot = installroot + prefix
wxpythonroot = installroot + sitepackages

try:
    if not os.path.exists(wxroot):
        os.makedirs(wxroot)
    
    if not os.path.exists(wxpythonroot):
        os.makedirs(wxpythonroot)
    
    for wildcard in ["*.py", "*.so", "*.dylib"]:
        files = glob.glob(os.path.join(wxwebkit_dir, wildcard))
        for afile in files:
            shutil.copy(afile, wxpythonroot)
    
    if sys.platform.startswith("darwin"):
        dylib_path = os.path.join(wxpythonroot, "libwxwebkit.dylib")
        os.system("install_name_tool -id %s %s" % (os.path.join(prefix, "libwxwebkit.dylib"), dylib_path))
        mac_update_dependencies(dylib_path, prefix)
        mac_update_dependencies(os.path.join(wxpythonroot, "_webview.so"), prefix)
        
        demodir = installroot + "/Applications/wxWebKit/Demos"
        if not os.path.exists(demodir):
            os.makedirs(demodir)
            
        shutil.copy(os.path.join(wxwk_root, "WebKit", "wx", "bindings", "python", "samples", "simple.py"), demodir)
        
        if os.path.exists(pkgname + ".pkg"):
            shutil.rmtree(pkgname + ".pkg")
    
        pkg_args = ['--title ' + pkgname,
                    '--out %s.pkg' % pkgname,
                    '--version ' + wxwk_version.strip(),
                    '--id org.wxwebkit.wxwebkit',
                    '--domain system',
                    '--root-volume-only',
                    '--root ' + installroot,
                    '--resources %s/mac/resources' % script_dir,
                    '--verbose'
                    ]
    
        packagemaker = "/Developer/Applications/Utilities/PackageMaker.app/Contents/MacOS/PackageMaker"
        exitIfError(packagemaker + " %s" % (string.join(pkg_args, " ")))
finally:
    if os.path.exists(tempdir):
        shutil.rmtree(tempdir)

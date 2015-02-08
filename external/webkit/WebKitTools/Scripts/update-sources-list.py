#!/usr/bin/env python
import os, sys
from xml.dom import minidom

scriptDir = os.path.abspath(sys.path[0])
wkroot = os.path.abspath(os.path.join(scriptDir, "../.."))

def getWebCoreFilesDict():
    """
    This method parses the WebCoreSources.bkl file, which has a list of all sources not specific
    to any port, and returns the result as a dictionary with items of the form
    (groupName, groupFiles). 
    """
    sources = {}
    sources_prefix = "WEBCORE_"
    filepath = os.path.join(wkroot, "WebCore/WebCoreSources.bkl")
    assert(os.path.exists(filepath))
    
    doc = minidom.parse(filepath)
    for sourceGroup in doc.getElementsByTagName("set"):
        groupName = ""
        if sourceGroup.attributes.has_key("var"):
            groupName = sourceGroup.attributes["var"].value
            groupName = groupName.replace(sources_prefix, "")
            
        sourcesList = []
        for node in sourceGroup.childNodes:
            if node.nodeType == node.TEXT_NODE:
                sourcesText = node.nodeValue.strip()
                sourcesList = sourcesText.split("\n")
                
        assert(groupName != "")
        assert(sourcesList != [])
        
        sources[groupName] = sourcesList
        
    return sources

def generateWebCoreSourcesGTKAndQT(sources):
    """
    Convert the dictionary obtained from getWebCoreFilesDict() into a Unix makefile syntax,
    which IIUC is suitable for both GTK and QT build systems. To take advantage of this,
    QT and GTK would have to include the file "WebCore/sources.inc" into their makefiles.
    """
    makefileString = ""
    
    for key in sources.keys():
        makefileString += key + "+="
        for source in sources[key]:
            makefileString += " \\\n\t\t" + source.strip()
            
        makefileString += "\n\n"
    
    makefileString += "BASE_SOURCES +="
    for key in sources.keys():
        makefileString += " \\\n\t\t" + key
    
    outfile = os.path.join(wkroot, "WebCore/sources.inc")
    sourcefile = open(outfile, "w")
    sourcefile.write(makefileString)
    sourcefile.close()
    
sources = getWebCoreFilesDict()
generateWebCoreSourcesGTKAndQT(sources)

# Coming soon - MSVC and hopefully XCode support!

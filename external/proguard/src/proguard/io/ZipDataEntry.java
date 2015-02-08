
package proguard.io;

import proguard.classfile.ClassConstants;

import java.io.*;
import java.util.zip.*;

public class ZipDataEntry implements DataEntry
{
    private final DataEntry      parent;
    private final ZipEntry       zipEntry;
    private       ZipInputStream zipInputStream;


    public ZipDataEntry(DataEntry      parent,
                        ZipEntry       zipEntry,
                        ZipInputStream zipInputStream)
    {
        this.parent         = parent;
        this.zipEntry       = zipEntry;
        this.zipInputStream = zipInputStream;
    }


    // Implementations for DataEntry.

    public String getName()
    {
        // Get the right separators.
        String name = zipEntry.getName()
            .replace(File.separatorChar, ClassConstants.INTERNAL_PACKAGE_SEPARATOR);

        // Chop the trailing directory slash, if any.
        int length = name.length();
        return length > 0 &&
               name.charAt(length-1) == ClassConstants.INTERNAL_PACKAGE_SEPARATOR ?
                   name.substring(0, length -1) :
                   name;
    }


    public boolean isDirectory()
    {
        return zipEntry.isDirectory();
    }


    public InputStream getInputStream() throws IOException
    {
        return zipInputStream;
    }


    public void closeInputStream() throws IOException
    {
        zipInputStream.closeEntry();
        zipInputStream = null;
    }


    public DataEntry getParent()
    {
        return parent;
    }


    // Implementations for Object.

    public String toString()
    {
        return parent.toString() + ':' + getName();
    }
}

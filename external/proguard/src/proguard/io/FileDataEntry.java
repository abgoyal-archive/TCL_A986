
package proguard.io;

import proguard.classfile.ClassConstants;

import java.io.*;

public class FileDataEntry implements DataEntry
{
    private final File        directory;
    private final File        file;
    private InputStream inputStream;


    public FileDataEntry(File directory,
                         File file)
    {
        this.directory = directory;
        this.file      = file;
    }


    // Implementations for DataEntry.

    public String getName()
    {
        // Chop the directory name from the file name and get the right separators.
        return file.equals(directory) ?
            file.getName() :
            file.getPath()
            .substring(directory.getPath().length() + File.separator.length())
            .replace(File.separatorChar, ClassConstants.INTERNAL_PACKAGE_SEPARATOR);
    }


    public boolean isDirectory()
    {
        return file.isDirectory();
    }


    public InputStream getInputStream() throws IOException
    {
        if (inputStream == null)
        {
            inputStream = new BufferedInputStream(new FileInputStream(file));
        }

        return inputStream;
    }


    public void closeInputStream() throws IOException
    {
        inputStream.close();
        inputStream = null;
    }


    public DataEntry getParent()
    {
        return null;
    }


    // Implementations for Object.

    public String toString()
    {
        return getName();
    }
}

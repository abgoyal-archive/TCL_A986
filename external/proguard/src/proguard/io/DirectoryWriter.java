
package proguard.io;

import proguard.classfile.ClassConstants;

import java.io.*;


public class DirectoryWriter implements DataEntryWriter
{
    private final File    baseFile;
    private final boolean isFile;

    private File         currentFile;
    private OutputStream currentOutputStream;
    private Finisher     currentFinisher;


    /**
     * Creates a new DirectoryWriter.
     * @param baseFile the base directory to which all files will be written.
     */
    public DirectoryWriter(File    baseFile,
                           boolean isFile)
    {
        this.baseFile = baseFile;
        this.isFile   = isFile;
    }


    // Implementations for DataEntryWriter.

    public boolean createDirectory(DataEntry dataEntry) throws IOException
    {
        // Should we close the current file?
        if (!isFile &&
            currentFile != null)
        {
            closeEntry();
        }

        File directory = getFile(dataEntry);
        if (!directory.exists() &&
            !directory.mkdirs())
        {
            throw new IOException("Can't create directory [" + directory.getPath() + "]");
        }

        return true;
    }


    public OutputStream getOutputStream(DataEntry dataEntry) throws IOException
    {
        return getOutputStream(dataEntry,  null);
    }


    public OutputStream getOutputStream(DataEntry dataEntry,
                                        Finisher  finisher) throws IOException
    {
        File file = getFile(dataEntry);

        // Should we close the current file?
        if (!isFile             &&
            currentFile != null &&
            !currentFile.equals(file))
        {
            closeEntry();
        }

        // Do we need a new stream?
        if (currentOutputStream == null)
        {
            // Make sure the parent directories exist.
            File parentDirectory = file.getParentFile();
            if (parentDirectory != null   &&
                !parentDirectory.exists() &&
                !parentDirectory.mkdirs())
            {
                throw new IOException("Can't create directory [" + parentDirectory.getPath() + "]");
            }

            // Open a new output stream for writing to the file.
            currentOutputStream =
                new BufferedOutputStream(
                new FileOutputStream(file));

            currentFinisher = finisher;
            currentFile     = file;
        }

        return currentOutputStream;
    }


    public void close() throws IOException
    {
        // Close the file stream, if any.
        closeEntry();
    }


    // Small utility methods.

    /**
     * Returns the file for the given data entry.
     */
    private File getFile(DataEntry dataEntry)
    {
        // Use the specified file, or construct a new file.
        return isFile ?
            baseFile :
            new File(baseFile,
                     dataEntry.getName().replace(ClassConstants.INTERNAL_PACKAGE_SEPARATOR,
                                                 File.separatorChar));
    }


    /**
     * Closes the previous file, if any.
     */
    private void closeEntry() throws IOException
    {
        // Close the file stream, if any.
        if (currentOutputStream != null)
        {
            // Let any finisher finish up first.
            if (currentFinisher != null)
            {
                currentFinisher.finish();
                currentFinisher = null;
            }

            currentOutputStream.close();
            currentOutputStream = null;
            currentFile         = null;
        }
    }
}


package proguard.io;

import java.io.*;


public class DirectoryPump implements DataEntryPump
{
    private final File directory;


    public DirectoryPump(File directory)
    {
        this.directory = directory;
    }


    // Implementations for DataEntryPump.

    public void pumpDataEntries(DataEntryReader dataEntryReader)
    throws IOException
    {
        if (!directory.exists())
        {
            throw new IOException("No such file or directory");
        }

        readFiles(directory, dataEntryReader);
    }


    /**
     * Reads the given subdirectory recursively, applying the given DataEntryReader
     * to all files that are encountered.
     */
    private void readFiles(File file, DataEntryReader dataEntryReader)
    throws IOException
    {
        // Pass the file data entry to the reader.
        dataEntryReader.read(new FileDataEntry(directory, file));

        if (file.isDirectory())
        {
            // Recurse into the subdirectory.
            File[] files = file.listFiles();

            for (int index = 0; index < files.length; index++)
            {
                readFiles(files[index], dataEntryReader);
            }
        }
    }
}

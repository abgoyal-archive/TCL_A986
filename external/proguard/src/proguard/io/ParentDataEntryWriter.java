
package proguard.io;

import java.io.*;

public class ParentDataEntryWriter implements DataEntryWriter
{
    private DataEntryWriter dataEntryWriter;


    /**
     * Creates a new ParentDataEntryWriter.
     * @param dataEntryWriter the DataEntryWriter to which the writing will be
     *                        delegated, passing the data entries' parents.
     */
    public ParentDataEntryWriter(DataEntryWriter dataEntryWriter)
    {
        this.dataEntryWriter = dataEntryWriter;
    }


    // Implementations for DataEntryWriter.


    public boolean createDirectory(DataEntry dataEntry) throws IOException
    {
        return getOutputStream(dataEntry) != null;
    }


    public OutputStream getOutputStream(DataEntry dataEntry) throws IOException
    {
        return getOutputStream(dataEntry, null);
    }


    public OutputStream getOutputStream(DataEntry dataEntry,
                                        Finisher  finisher) throws IOException
    {
        return dataEntryWriter.getOutputStream(dataEntry.getParent(),
                                               finisher);
    }


    public void close() throws IOException
    {
        dataEntryWriter.close();
        dataEntryWriter = null;
    }
}

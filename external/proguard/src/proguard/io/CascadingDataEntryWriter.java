
package proguard.io;

import java.io.*;

public class CascadingDataEntryWriter implements DataEntryWriter
{
    private DataEntryWriter dataEntryWriter1;
    private DataEntryWriter dataEntryWriter2;


    /**
     * Creates a new CascadingDataEntryWriter.
     * @param dataEntryWriter1 the DataEntryWriter to which the writing will be
     *                         delegated first.
     * @param dataEntryWriter2 the DataEntryWriter to which the writing will be
     *                         delegated, if the first one can't provide an
     *                         output stream.
     */
    public CascadingDataEntryWriter(DataEntryWriter dataEntryWriter1,
                                    DataEntryWriter dataEntryWriter2)
    {
        this.dataEntryWriter1 = dataEntryWriter1;
        this.dataEntryWriter2 = dataEntryWriter2;
    }


    // Implementations for DataEntryWriter.


    public boolean createDirectory(DataEntry dataEntry) throws IOException
    {
        // Try to create a directory with the first data entry writer, or
        // otherwise with the second data entry writer.
        return dataEntryWriter1.createDirectory(dataEntry) ||
               dataEntryWriter2.createDirectory(dataEntry);
    }


    public OutputStream getOutputStream(DataEntry dataEntry) throws IOException
    {
        return getOutputStream(dataEntry,  null);
    }


    public OutputStream getOutputStream(DataEntry dataEntry,
                                        Finisher  finisher) throws IOException
    {
        // Try to get an output stream from the first data entry writer.
        OutputStream outputStream =
            dataEntryWriter1.getOutputStream(dataEntry, finisher);

        // Return it, if it's not null. Otherwise try to get an output stream
        // from the second data entry writer.
        return outputStream != null ?
            outputStream :
            dataEntryWriter2.getOutputStream(dataEntry, finisher);
    }


    public void close() throws IOException
    {
        dataEntryWriter1.close();
        dataEntryWriter2.close();

        dataEntryWriter1 = null;
        dataEntryWriter2 = null;
    }
}

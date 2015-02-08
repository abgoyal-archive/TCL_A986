
package proguard.io;

import java.io.*;

public class FilteredDataEntryWriter implements DataEntryWriter
{
    private final DataEntryFilter dataEntryFilter;
    private DataEntryWriter acceptedDataEntryWriter;
    private DataEntryWriter rejectedDataEntryWriter;


    /**
     * Creates a new FilteredDataEntryWriter with only a writer for accepted
     * data entries.
     * @param dataEntryFilter         the data entry filter.
     * @param acceptedDataEntryWriter the DataEntryWriter to which the writing
     *                                will be delegated if the filter accepts
     *                                the data entry. May be <code>null</code>.
     */
    public FilteredDataEntryWriter(DataEntryFilter dataEntryFilter,
                                   DataEntryWriter acceptedDataEntryWriter)
    {
        this(dataEntryFilter, acceptedDataEntryWriter, null);
    }


    /**
     * Creates a new FilteredDataEntryWriter.
     * @param dataEntryFilter         the data entry filter.
     * @param acceptedDataEntryWriter the DataEntryWriter to which the writing
     *                                will be delegated if the filter accepts
     *                                the data entry. May be <code>null</code>.
     * @param rejectedDataEntryWriter the DataEntryWriter to which the writing
     *                                will be delegated if the filter does not
     *                                accept the data entry. May be
     *                                <code>null</code>.
     */
    public FilteredDataEntryWriter(DataEntryFilter dataEntryFilter,
                                   DataEntryWriter acceptedDataEntryWriter,
                                   DataEntryWriter rejectedDataEntryWriter)
    {
        this.dataEntryFilter         = dataEntryFilter;
        this.acceptedDataEntryWriter = acceptedDataEntryWriter;
        this.rejectedDataEntryWriter = rejectedDataEntryWriter;
    }


    // Implementations for DataEntryWriter.

    public boolean createDirectory(DataEntry dataEntry) throws IOException
    {
        // Get the right data entry writer.
        DataEntryWriter dataEntryWriter = dataEntryFilter.accepts(dataEntry) ?
            acceptedDataEntryWriter :
            rejectedDataEntryWriter;

        // Delegate to it, if it's not null.
        return dataEntryWriter != null &&
               dataEntryWriter.createDirectory(dataEntry);
    }


    public OutputStream getOutputStream(DataEntry dataEntry) throws IOException
    {
        return getOutputStream(dataEntry,  null);
    }


    public OutputStream getOutputStream(DataEntry dataEntry,
                                        Finisher  finisher) throws IOException
    {
        // Get the right data entry writer.
        DataEntryWriter dataEntryWriter = dataEntryFilter.accepts(dataEntry) ?
            acceptedDataEntryWriter :
            rejectedDataEntryWriter;

        // Delegate to it, if it's not null.
        return dataEntryWriter != null ?
            dataEntryWriter.getOutputStream(dataEntry, finisher) :
            null;
    }


    public void close() throws IOException
    {
        if (acceptedDataEntryWriter != null)
        {
            acceptedDataEntryWriter.close();
            acceptedDataEntryWriter = null;
        }

        if (rejectedDataEntryWriter != null)
        {
            rejectedDataEntryWriter.close();
            rejectedDataEntryWriter = null;
        }
    }
}

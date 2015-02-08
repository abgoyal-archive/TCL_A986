
package proguard.io;

import java.io.IOException;


public class FilteredDataEntryReader implements DataEntryReader
{
    private final DataEntryFilter dataEntryFilter;
    private final DataEntryReader acceptedDataEntryReader;
    private final DataEntryReader rejectedDataEntryReader;


    /**
     * Creates a new FilteredDataEntryReader with only a reader for accepted
     * data entries.
     * @param dataEntryFilter         the data entry filter.
     * @param acceptedDataEntryReader the DataEntryReader to which the reading
     *                                will be delegated if the filter accepts
     *                                the data entry. May be <code>null</code>.
     */
    public FilteredDataEntryReader(DataEntryFilter dataEntryFilter,
                                   DataEntryReader acceptedDataEntryReader)
    {
        this(dataEntryFilter, acceptedDataEntryReader, null);
    }


    /**
     * Creates a new FilteredDataEntryReader.
     * @param dataEntryFilter         the data entry filter.
     * @param acceptedDataEntryReader the DataEntryReader to which the reading
     *                                will be delegated if the filter accepts
     *                                the data entry. May be <code>null</code>.
     * @param rejectedDataEntryReader the DataEntryReader to which the reading
     *                                will be delegated if the filter does not
     *                                accept the data entry. May be
     *                                <code>null</code>.
     */
    public FilteredDataEntryReader(DataEntryFilter dataEntryFilter,
                                   DataEntryReader acceptedDataEntryReader,
                                   DataEntryReader rejectedDataEntryReader)
    {
        this.dataEntryFilter         = dataEntryFilter;
        this.acceptedDataEntryReader = acceptedDataEntryReader;
        this.rejectedDataEntryReader = rejectedDataEntryReader;
    }


    // Implementations for DataEntryReader.

    public void read(DataEntry dataEntry)
    throws IOException
    {
        DataEntryReader dataEntryReader = dataEntryFilter.accepts(dataEntry) ?
            acceptedDataEntryReader :
            rejectedDataEntryReader;

        if (dataEntryReader != null)
        {
            dataEntryReader.read(dataEntry);
        }
    }
}

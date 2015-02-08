
package proguard.io;

public class DataEntryParentFilter
implements   DataEntryFilter
{
    private final DataEntryFilter dataEntryFilter;


    /**
     * Creates a new ParentFilter.
     * @param dataEntryFilter the filter that will be applied to the data
     *                        entry's parent.
     */
    public DataEntryParentFilter(DataEntryFilter dataEntryFilter)
    {
        this.dataEntryFilter = dataEntryFilter;
    }


    // Implementations for DataEntryFilter.

    public boolean accepts(DataEntry dataEntry)
    {
        return dataEntry != null && dataEntryFilter.accepts(dataEntry.getParent());
    }
}

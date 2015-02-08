
package proguard.io;


public interface DataEntryFilter
{
    /**
     * Checks whether the filter accepts the given data entry.
     * @param dataEntry the data entry to filter.
     * @return a boolean indicating whether the filter accepts the given data
     *         entry.
     */
    public boolean accepts(DataEntry dataEntry);
}

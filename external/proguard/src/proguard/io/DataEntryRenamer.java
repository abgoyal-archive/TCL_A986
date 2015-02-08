
package proguard.io;

import proguard.classfile.ClassConstants;

import java.io.IOException;
import java.util.Map;

public class DataEntryRenamer implements DataEntryReader
{
    private final Map             nameMap;
    private final DataEntryReader renamedDataEntryReader;
    private final DataEntryReader missingDataEntryReader;


    /**
     * Creates a new DataEntryRenamer.
     * @param nameMap                the map from old names to new names.
     * @param renamedDataEntryReader the DataEntryReader to which renamed data
     *                               entries will be passed.
     */
    public DataEntryRenamer(Map             nameMap,
                            DataEntryReader renamedDataEntryReader)
    {
        this(nameMap, renamedDataEntryReader, null);
    }


    /**
     * Creates a new DataEntryRenamer.
     * @param nameMap                the map from old names to new names.
     * @param renamedDataEntryReader the DataEntryReader to which renamed data
     *                               entries will be passed.
     * @param missingDataEntryReader the optional DataEntryReader to which data
     *                               entries that can't be renamed will be
     *                               passed.
     */
    public DataEntryRenamer(Map             nameMap,
                            DataEntryReader renamedDataEntryReader,
                            DataEntryReader missingDataEntryReader)
    {
        this.nameMap                = nameMap;
        this.renamedDataEntryReader = renamedDataEntryReader;
        this.missingDataEntryReader = missingDataEntryReader;
    }


    // Implementations for DataEntryReader.

    public void read(DataEntry dataEntry) throws IOException
    {
        String name = dataEntry.getName();

        // Add a directory separator if necessary.
        if (dataEntry.isDirectory() &&
            name.length() > 0)
        {
            name += ClassConstants.INTERNAL_PACKAGE_SEPARATOR;
        }

        String newName = (String)nameMap.get(name);
        if (newName != null)
        {
            // Add remove the directory separator if necessary.
            if (dataEntry.isDirectory() &&
                newName.length() > 0)
            {
                newName = newName.substring(0, newName.length() -  1);
            }

            renamedDataEntryReader.read(new RenamedDataEntry(dataEntry, newName));
        }
        else if (missingDataEntryReader != null)
        {
            missingDataEntryReader.read(dataEntry);
        }
    }
}
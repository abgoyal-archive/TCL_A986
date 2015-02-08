
package proguard.io;

import proguard.classfile.ClassConstants;
import proguard.util.ExtensionMatcher;

import java.io.IOException;


public class DirectoryFilter extends FilteredDataEntryReader
{
    /**
     * Creates a new ClassFilter that delegates reading directories to the
     * given reader.
     */
    public DirectoryFilter(DataEntryReader directoryReader)
    {
        this (directoryReader, null);
    }


    /**
     * Creates a new ClassFilter that delegates to either of the two given
     * readers.
     */
    public DirectoryFilter(DataEntryReader directoryReader,
                           DataEntryReader otherReader)
    {
        super(new DataEntryDirectoryFilter(),
              directoryReader,
              otherReader);
    }
}
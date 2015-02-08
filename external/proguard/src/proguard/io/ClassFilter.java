
package proguard.io;

import proguard.classfile.ClassConstants;
import proguard.util.ExtensionMatcher;

import java.io.IOException;


public class ClassFilter extends FilteredDataEntryReader
{
    /**
     * Creates a new ClassFilter that delegates reading classes to the
     * given reader.
     */
    public ClassFilter(DataEntryReader classReader)
    {
        this(classReader, null);
    }


    /**
     * Creates a new ClassFilter that delegates to either of the two given
     * readers.
     */
    public ClassFilter(DataEntryReader classReader,
                       DataEntryReader dataEntryReader)
    {
        super(new DataEntryNameFilter(
              new ExtensionMatcher(ClassConstants.CLASS_FILE_EXTENSION)),
              classReader,
              dataEntryReader);
    }
}


package proguard.io;

import proguard.util.*;

import java.util.List;

public class NameFilter extends FilteredDataEntryReader
{
    /**
     * Creates a new NameFilter that delegates to the given reader, depending
     * on the given list of filters.
     */
    public NameFilter(String          regularExpression,
                      DataEntryReader acceptedDataEntryReader)
    {
        this(regularExpression, acceptedDataEntryReader, null);
    }


    /**
     * Creates a new NameFilter that delegates to either of the two given
     * readers, depending on the given list of filters.
     */
    public NameFilter(String          regularExpression,
                      DataEntryReader acceptedDataEntryReader,
                      DataEntryReader rejectedDataEntryReader)
    {
        super(new DataEntryNameFilter(new ListParser(new FileNameParser()).parse(regularExpression)),
              acceptedDataEntryReader,
              rejectedDataEntryReader);
    }


    /**
     * Creates a new NameFilter that delegates to the given reader, depending
     * on the given list of filters.
     */
    public NameFilter(List            regularExpressions,
                      DataEntryReader acceptedDataEntryReader)
    {
        this(regularExpressions, acceptedDataEntryReader, null);
    }


    /**
     * Creates a new NameFilter that delegates to either of the two given
     * readers, depending on the given list of filters.
     */
    public NameFilter(List            regularExpressions,
                      DataEntryReader acceptedDataEntryReader,
                      DataEntryReader rejectedDataEntryReader)
    {
        super(new DataEntryNameFilter(new ListParser(new FileNameParser()).parse(regularExpressions)),
              acceptedDataEntryReader,
              rejectedDataEntryReader);
    }
}
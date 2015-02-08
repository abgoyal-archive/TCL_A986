
package proguard;

import proguard.io.*;
import proguard.util.*;

import java.util.List;


public class DataEntryReaderFactory
{
    /**
     * Creates a DataEntryReader that can read the given class path entry.
     *
     * @param messagePrefix  a prefix for messages that are printed out.
     * @param classPathEntry the input class path entry.
     * @param reader         a data entry reader to which the reading of actual
     *                       classes and resource files can be delegated.
     * @return a DataEntryReader for reading the given class path entry.
     */
    public static DataEntryReader createDataEntryReader(String          messagePrefix,
                                                        ClassPathEntry  classPathEntry,
                                                        DataEntryReader reader)
    {
        String entryName = classPathEntry.getName();
        boolean isJar = endsWithIgnoreCase(entryName, ".jar");
        boolean isWar = endsWithIgnoreCase(entryName, ".war");
        boolean isEar = endsWithIgnoreCase(entryName, ".ear");
        boolean isZip = endsWithIgnoreCase(entryName, ".zip");

        List filter    = classPathEntry.getFilter();
        List jarFilter = classPathEntry.getJarFilter();
        List warFilter = classPathEntry.getWarFilter();
        List earFilter = classPathEntry.getEarFilter();
        List zipFilter = classPathEntry.getZipFilter();

        System.out.println(messagePrefix +
                           (isJar ? "jar" :
                            isWar ? "war" :
                            isEar ? "ear" :
                            isZip ? "zip" :
                                    "directory") +
                           " [" + classPathEntry.getName() + "]" +
                           (filter    != null ||
                            jarFilter != null ||
                            warFilter != null ||
                            earFilter != null ||
                            zipFilter != null ? " (filtered)" : ""));

        // Add a filter, if specified.
        if (filter != null)
        {
            reader = new FilteredDataEntryReader(
                     new DataEntryNameFilter(
                     new ListParser(new FileNameParser()).parse(filter)),
                         reader);
        }

        // Unzip any jars, if necessary.
        reader = wrapInJarReader(reader, isJar, jarFilter, ".jar");
        if (!isJar)
        {
            // Unzip any wars, if necessary.
            reader = wrapInJarReader(reader, isWar, warFilter, ".war");
            if (!isWar)
            {
                // Unzip any ears, if necessary.
                reader = wrapInJarReader(reader, isEar, earFilter, ".ear");
                if (!isEar)
                {
                    // Unzip any zips, if necessary.
                    reader = wrapInJarReader(reader, isZip, zipFilter, ".zip");
                }
            }
        }

        return reader;
    }


    /**
     *  Wraps the given DataEntryReader in a JarReader, filtering it if necessary.
     */
    private static DataEntryReader wrapInJarReader(DataEntryReader reader,
                                                   boolean         isJar,
                                                   List            jarFilter,
                                                   String          jarExtension)
    {
        // Unzip any jars, if necessary.
        DataEntryReader jarReader = new JarReader(reader);

        if (isJar)
        {
            // Always unzip.
            return jarReader;
        }
        else
        {
            // Add a filter, if specified.
            if (jarFilter != null)
            {
                jarReader = new FilteredDataEntryReader(
                            new DataEntryNameFilter(
                            new ListParser(new FileNameParser()).parse(jarFilter)),
                                jarReader);
            }

            // Only unzip the right type of jars.
            return new FilteredDataEntryReader(
                   new DataEntryNameFilter(
                   new ExtensionMatcher(jarExtension)),
                       jarReader,
                       reader);
        }
    }


    /**
     * Returns whether the given string ends with the given suffix, ignoring its
     * case.
     */
    private static boolean endsWithIgnoreCase(String string, String suffix)
    {
        int stringLength = string.length();
        int suffixLength = suffix.length();

        return string.regionMatches(true, stringLength - suffixLength, suffix, 0, suffixLength);
    }
}

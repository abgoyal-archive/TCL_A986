
package proguard.io;

import proguard.classfile.*;
import proguard.classfile.io.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.ClassVisitor;

import java.io.*;

public class ClassReader implements DataEntryReader
{
    private final boolean        isLibrary;
    private final boolean        skipNonPublicLibraryClasses;
    private final boolean        skipNonPublicLibraryClassMembers;
    private final WarningPrinter warningPrinter;
    private final ClassVisitor   classVisitor;


    /**
     * Creates a new DataEntryClassFilter for reading the specified
     * Clazz objects.
     */
    public ClassReader(boolean        isLibrary,
                       boolean        skipNonPublicLibraryClasses,
                       boolean        skipNonPublicLibraryClassMembers,
                       WarningPrinter warningPrinter,
                       ClassVisitor   classVisitor)
    {
        this.isLibrary                        = isLibrary;
        this.skipNonPublicLibraryClasses      = skipNonPublicLibraryClasses;
        this.skipNonPublicLibraryClassMembers = skipNonPublicLibraryClassMembers;
        this.warningPrinter                   = warningPrinter;
        this.classVisitor                     = classVisitor;
    }


    // Implementations for DataEntryReader.

    public void read(DataEntry dataEntry) throws IOException
    {
        try
        {
            // Get the input stream.
            InputStream inputStream = dataEntry.getInputStream();

            // Wrap it into a data input stream.
            DataInputStream dataInputStream = new DataInputStream(inputStream);

            // Create a Clazz representation.
            Clazz clazz;
            if (isLibrary)
            {
                clazz = new LibraryClass();
                clazz.accept(new LibraryClassReader(dataInputStream, skipNonPublicLibraryClasses, skipNonPublicLibraryClassMembers));
            }
            else
            {
                clazz = new ProgramClass();
                clazz.accept(new ProgramClassReader(dataInputStream));
            }

            // Apply the visitor, if we have a real class.
            String className = clazz.getName();
            if (className != null)
            {
                if (!dataEntry.getName().replace(File.pathSeparatorChar, ClassConstants.INTERNAL_PACKAGE_SEPARATOR).equals(className+ClassConstants.CLASS_FILE_EXTENSION) &&
                    warningPrinter != null)
                {
                    warningPrinter.print(className,
                                         "Warning: class [" + dataEntry.getName() + "] unexpectedly contains class [" + ClassUtil.externalClassName(className) + "]");
                }

                clazz.accept(classVisitor);
            }

            dataEntry.closeInputStream();
        }
        catch (Exception ex)
        {
            throw new IOException("Can't process class ["+dataEntry.getName()+"] ("+ex.getMessage()+")");
        }
    }
}

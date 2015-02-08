
package proguard.io;

import proguard.classfile.*;
import proguard.classfile.io.ProgramClassWriter;

import java.io.*;


public class ClassRewriter implements DataEntryReader
{
    private final ClassPool       classPool;
    private final DataEntryWriter dataEntryWriter;


    public ClassRewriter(ClassPool       classPool,
                         DataEntryWriter dataEntryWriter)
    {
        this.classPool       = classPool;
        this.dataEntryWriter = dataEntryWriter;
    }


    // Implementations for DataEntryReader.

    public void read(DataEntry dataEntry) throws IOException
    {
        String inputName = dataEntry.getName();
        String className = inputName.substring(0, inputName.length() - ClassConstants.CLASS_FILE_EXTENSION.length());

        // Find the modified class corrsponding to the input entry.
        ProgramClass programClass = (ProgramClass)classPool.getClass(className);
        if (programClass != null)
        {
            // Rename the data entry if necessary.
            String newClassName = programClass.getName();
            if (!className.equals(newClassName))
            {
                dataEntry = new RenamedDataEntry(dataEntry, newClassName + ClassConstants.CLASS_FILE_EXTENSION);
            }

            // Get the output entry corresponding to this input entry.
            OutputStream outputStream = dataEntryWriter.getOutputStream(dataEntry);
            if (outputStream != null)
            {
                // Write the class to the output entry.
                DataOutputStream classOutputStream = new DataOutputStream(outputStream);

                new ProgramClassWriter(classOutputStream).visitProgramClass(programClass);

                classOutputStream.flush();
            }
        }
    }
}

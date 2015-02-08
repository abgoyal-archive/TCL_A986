
package proguard.classfile.editor;

import proguard.classfile.attribute.ExceptionsAttribute;

public class ExceptionsAttributeEditor
{
    private ExceptionsAttribute targetExceptionsAttribute;


    /**
     * Creates a new ExceptionsAttributeEditor that will edit exceptions in the
     * given exceptions attribute.
     */
    public ExceptionsAttributeEditor(ExceptionsAttribute targetExceptionsAttribute)
    {
        this.targetExceptionsAttribute = targetExceptionsAttribute;
    }


    /**
     * Adds a given exception to the exceptions attribute.
     */
    public void addException(int exceptionIndex)
    {
        int   exceptionIndexTableLength = targetExceptionsAttribute.u2exceptionIndexTableLength;
        int[] exceptionIndexTable       = targetExceptionsAttribute.u2exceptionIndexTable;

        // Make sure there is enough space for the new exception.
        if (exceptionIndexTable.length <= exceptionIndexTableLength)
        {
            targetExceptionsAttribute.u2exceptionIndexTable = new int[exceptionIndexTableLength+1];
            System.arraycopy(exceptionIndexTable, 0,
                             targetExceptionsAttribute.u2exceptionIndexTable, 0,
                             exceptionIndexTableLength);
            exceptionIndexTable = targetExceptionsAttribute.u2exceptionIndexTable;
        }

        // Add the exception.
        exceptionIndexTable[targetExceptionsAttribute.u2exceptionIndexTableLength++] = exceptionIndex;
    }
}

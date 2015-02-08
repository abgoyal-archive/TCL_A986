
package proguard.classfile.editor;

import proguard.classfile.attribute.*;

public class LocalVariableTableAttributeEditor
{
    private LocalVariableTableAttribute targetLocalVariableTableAttribute;


    /**
     * Creates a new LocalVariableTableAttributeEditor that will edit line numbers
     * in the given line number table attribute.
     */
    public LocalVariableTableAttributeEditor(LocalVariableTableAttribute targetLocalVariableTableAttribute)
    {
        this.targetLocalVariableTableAttribute = targetLocalVariableTableAttribute;
    }


    /**
     * Adds a given line number to the line number table attribute.
     */
    public void addLocalVariableInfo(LocalVariableInfo localVariableInfo)
    {
        int                 localVariableTableLength = targetLocalVariableTableAttribute.u2localVariableTableLength;
        LocalVariableInfo[] localVariableTable       = targetLocalVariableTableAttribute.localVariableTable;

        // Make sure there is enough space for the new localVariableInfo.
        if (localVariableTable.length <= localVariableTableLength)
        {
            targetLocalVariableTableAttribute.localVariableTable = new LocalVariableInfo[localVariableTableLength+1];
            System.arraycopy(localVariableTable, 0,
                             targetLocalVariableTableAttribute.localVariableTable, 0,
                             localVariableTableLength);
            localVariableTable = targetLocalVariableTableAttribute.localVariableTable;
        }

        // Add the localVariableInfo.
        localVariableTable[targetLocalVariableTableAttribute.u2localVariableTableLength++] = localVariableInfo;
    }
}
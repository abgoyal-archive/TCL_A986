
package proguard.classfile.editor;

import proguard.classfile.attribute.*;

public class LocalVariableTypeTableAttributeEditor
{
    private LocalVariableTypeTableAttribute targetLocalVariableTypeTableAttribute;


    /**
     * Creates a new LocalVariableTypeTableAttributeEditor that will edit line numbers
     * in the given line number table attribute.
     */
    public LocalVariableTypeTableAttributeEditor(LocalVariableTypeTableAttribute targetLocalVariableTypeTableAttribute)
    {
        this.targetLocalVariableTypeTableAttribute = targetLocalVariableTypeTableAttribute;
    }


    /**
     * Adds a given line number to the line number table attribute.
     */
    public void addLocalVariableTypeInfo(LocalVariableTypeInfo localVariableTypeInfo)
    {
        int                     localVariableTypeTableLength = targetLocalVariableTypeTableAttribute.u2localVariableTypeTableLength;
        LocalVariableTypeInfo[] localVariableTypeTable       = targetLocalVariableTypeTableAttribute.localVariableTypeTable;

        // Make sure there is enough space for the new localVariableTypeInfo.
        if (localVariableTypeTable.length <= localVariableTypeTableLength)
        {
            targetLocalVariableTypeTableAttribute.localVariableTypeTable = new LocalVariableTypeInfo[localVariableTypeTableLength+1];
            System.arraycopy(localVariableTypeTable, 0,
                             targetLocalVariableTypeTableAttribute.localVariableTypeTable, 0,
                             localVariableTypeTableLength);
            localVariableTypeTable = targetLocalVariableTypeTableAttribute.localVariableTypeTable;
        }

        // Add the localVariableTypeInfo.
        localVariableTypeTable[targetLocalVariableTypeTableAttribute.u2localVariableTypeTableLength++] = localVariableTypeInfo;
    }
}
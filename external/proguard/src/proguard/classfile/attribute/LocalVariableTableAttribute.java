
package proguard.classfile.attribute;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.*;

public class LocalVariableTableAttribute extends Attribute
{
    public int                 u2localVariableTableLength;
    public LocalVariableInfo[] localVariableTable;


    /**
     * Creates an uninitialized LocalVariableTableAttribute.
     */
    public LocalVariableTableAttribute()
    {
    }


    /**
     * Creates an initialized LocalVariableTableAttribute.
     */
    public LocalVariableTableAttribute(int                 u2attributeNameIndex,
                                       int                 u2localVariableTableLength,
                                       LocalVariableInfo[] localVariableTable)
    {
        super(u2attributeNameIndex);

        this.u2localVariableTableLength = u2localVariableTableLength;
        this.localVariableTable         = localVariableTable;
    }


    // Implementations for Attribute.

    public void accept(Clazz clazz, Method method, CodeAttribute codeAttribute, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitLocalVariableTableAttribute(clazz, method, codeAttribute, this);
    }


    /**
     * Applies the given visitor to all local variables.
     */
    public void localVariablesAccept(Clazz clazz, Method method, CodeAttribute codeAttribute, LocalVariableInfoVisitor localVariableInfoVisitor)
    {
        for (int index = 0; index < u2localVariableTableLength; index++)
        {
            // We don't need double dispatching here, since there is only one
            // type of LocalVariableInfo.
            localVariableInfoVisitor.visitLocalVariableInfo(clazz, method, codeAttribute, localVariableTable[index]);
        }
    }
}

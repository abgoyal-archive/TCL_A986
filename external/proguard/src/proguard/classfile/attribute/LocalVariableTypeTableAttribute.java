
package proguard.classfile.attribute;

import proguard.classfile.*;
import proguard.classfile.attribute.visitor.*;

public class LocalVariableTypeTableAttribute extends Attribute
{
    public int                     u2localVariableTypeTableLength;
    public LocalVariableTypeInfo[] localVariableTypeTable;


    /**
     * Creates an uninitialized LocalVariableTypeTableAttribute.
     */
    public LocalVariableTypeTableAttribute()
    {
    }


    /**
     * Creates an initialized LocalVariableTypeTableAttribute.
     */
    public LocalVariableTypeTableAttribute(int                     u2attributeNameIndex,
                                           int                     u2localVariableTypeTableLength,
                                           LocalVariableTypeInfo[] localVariableTypeTable)
    {
        super(u2attributeNameIndex);

        this.u2localVariableTypeTableLength = u2localVariableTypeTableLength;
        this.localVariableTypeTable         = localVariableTypeTable;
    }


    // Implementations for Attribute.

    public void accept(Clazz clazz, Method method, CodeAttribute codeAttribute, AttributeVisitor attributeVisitor)
    {
        attributeVisitor.visitLocalVariableTypeTableAttribute(clazz, method, codeAttribute, this);
    }


    /**
     * Applies the given visitor to all local variable types.
     */
    public void localVariablesAccept(Clazz clazz, Method method, CodeAttribute codeAttribute, LocalVariableTypeInfoVisitor localVariableTypeInfoVisitor)
    {
        for (int index = 0; index < u2localVariableTypeTableLength; index++)
        {
            // We don't need double dispatching here, since there is only one
            // type of LocalVariableTypeInfo.
            localVariableTypeInfoVisitor.visitLocalVariableTypeInfo(clazz, method, codeAttribute, localVariableTypeTable[index]);
        }
    }
}

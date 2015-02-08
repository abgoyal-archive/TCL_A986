
package proguard.classfile.editor;

import proguard.classfile.attribute.visitor.LocalVariableTypeInfoVisitor;
import proguard.classfile.attribute.*;
import proguard.classfile.*;

public class LocalVariableTypeInfoAdder
implements   LocalVariableTypeInfoVisitor
{
    private final ConstantAdder                     constantAdder;
    private final LocalVariableTypeTableAttributeEditor localVariableTypeTableAttributeEditor;


    /**
     * Creates a new LocalVariableTypeInfoAdder that will copy line numbers into the
     * given target line number table.
     */
    public LocalVariableTypeInfoAdder(ProgramClass                    targetClass,
                                      LocalVariableTypeTableAttribute targetLocalVariableTypeTableAttribute)
    {
        this.constantAdder                         = new ConstantAdder(targetClass);
        this.localVariableTypeTableAttributeEditor = new LocalVariableTypeTableAttributeEditor(targetLocalVariableTypeTableAttribute);
    }


    // Implementations for LocalVariableTypeInfoVisitor.

    public void visitLocalVariableTypeInfo(Clazz clazz, Method method, CodeAttribute codeAttribute, LocalVariableTypeInfo localVariableTypeInfo)
    {
        // Create a new line number.
        LocalVariableTypeInfo newLocalVariableTypeInfo =
            new LocalVariableTypeInfo(localVariableTypeInfo.u2startPC,
                                      localVariableTypeInfo.u2length,
                                      constantAdder.addConstant(clazz, localVariableTypeInfo.u2nameIndex),
                                      constantAdder.addConstant(clazz, localVariableTypeInfo.u2signatureIndex),
                                      localVariableTypeInfo.u2index);

        // TODO: Clone array.
        newLocalVariableTypeInfo.referencedClasses = localVariableTypeInfo.referencedClasses;

        // Add it to the target.
        localVariableTypeTableAttributeEditor.addLocalVariableTypeInfo(newLocalVariableTypeInfo);
    }
}
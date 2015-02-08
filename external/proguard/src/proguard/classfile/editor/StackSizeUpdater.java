
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.visitor.*;
import proguard.classfile.util.SimplifiedVisitor;

public class StackSizeUpdater
extends      SimplifiedVisitor
implements   AttributeVisitor
{
    private final StackSizeComputer stackSizeComputer = new StackSizeComputer();


    // Implementations for AttributeVisitor.

    public void visitAnyAttribute(Clazz clazz, Attribute attribute) {}


    public void visitCodeAttribute(Clazz clazz, Method method, CodeAttribute codeAttribute)
    {
        // Compute the stack sizes.
        stackSizeComputer.visitCodeAttribute(clazz, method, codeAttribute);

        // Update the maximum stack size.
        codeAttribute.u2maxStack = stackSizeComputer.getMaxStackSize();
    }
}

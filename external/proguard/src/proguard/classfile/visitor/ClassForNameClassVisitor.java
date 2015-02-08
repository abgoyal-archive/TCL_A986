
package proguard.classfile.visitor;

import proguard.classfile.Clazz;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;


public class ClassForNameClassVisitor
extends      SimplifiedVisitor
implements   ConstantVisitor
{
    private final ClassVisitor classVisitor;


    /**
     * Creates a new ClassHierarchyTraveler.
     * @param classVisitor the <code>ClassVisitor</code> to which visits will
     *                     be delegated.
     */
    public ClassForNameClassVisitor(ClassVisitor classVisitor)
    {
        this.classVisitor = classVisitor;
    }


    // Implementations for ConstantVisitor.

    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        // Visit the referenced class from the Class.forName construct, if any.
        stringConstant.referencedClassAccept(classVisitor);
    }
}


package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.util.SimplifiedVisitor;

public class ImplementedClassConstantFilter
extends      SimplifiedVisitor
implements   ConstantVisitor
{
    private final Clazz           implementedClass;
    private final ConstantVisitor constantVisitor;


    /**
     * Creates a new ImplementedClassConstantFilter.
     * @param implementedClass the class whose implementations will not be
     *                         visited.
     * @param constantVisitor  the <code>ConstantVisitor</code> to which visits
     *                         will be delegated.
     */
    public ImplementedClassConstantFilter(Clazz           implementedClass,
                                          ConstantVisitor constantVisitor)
    {
        this.implementedClass = implementedClass;
        this.constantVisitor  = constantVisitor;
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        Clazz referencedClass = classConstant.referencedClass;
        if (referencedClass == null ||
            !referencedClass.extendsOrImplements(implementedClass))
        {
            constantVisitor.visitClassConstant(clazz, classConstant);
        }
    }
}

package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;


public class ClassPoolFiller
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private final ClassPool classPool;


    /**
     * Creates a new ClassPoolFiller.
     */
    public ClassPoolFiller(ClassPool classPool)
    {
        this.classPool = classPool;
    }


    // Implementations for ClassVisitor.

    public void visitAnyClass(Clazz clazz)
    {
        classPool.addClass(clazz);
    }
}

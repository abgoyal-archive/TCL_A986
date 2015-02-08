
package proguard.classfile.util;

import proguard.classfile.*;
import proguard.classfile.constant.*;
import proguard.classfile.constant.visitor.ConstantVisitor;

public class StringReferenceInitializer
extends      SimplifiedVisitor
implements   ConstantVisitor
{
    private final ClassPool programClassPool;
    private final ClassPool libraryClassPool;


    /**
     * Creates a new StringReferenceInitializer.
     */
    public StringReferenceInitializer(ClassPool programClassPool,
                                      ClassPool libraryClassPool)
    {
        this.programClassPool = programClassPool;
        this.libraryClassPool = libraryClassPool;
    }


    // Implementations for ConstantVisitor.

    public void visitAnyConstant(Clazz clazz, Constant constant) {}


    public void visitStringConstant(Clazz clazz, StringConstant stringConstant)
    {
        if (stringConstant.referencedClass == null)
        {
            // See if we can find the referenced class.
            stringConstant.referencedClass =
                findClass(ClassUtil.internalClassName(stringConstant.getString(clazz)));
        }
    }


    // Small utility methods.

    /**
     * Returns the class with the given name, either for the program class pool
     * or from the library class pool, or <code>null</code> if it can't be found.
     */
    private Clazz findClass(String name)
    {
        // First look for the class in the program class pool.
        Clazz clazz = programClassPool.getClass(name);

        // Otherwise look for the class in the library class pool.
        if (clazz == null)
        {
            clazz = libraryClassPool.getClass(name);
        }

        return clazz;
    }
}

package proguard.classfile.constant.visitor;

import proguard.classfile.*;
import proguard.classfile.editor.*;
import proguard.classfile.attribute.*;
import proguard.classfile.attribute.annotation.*;
import proguard.classfile.attribute.preverification.*;
import proguard.classfile.constant.ClassConstant;
import proguard.classfile.constant.visitor.ConstantVisitor;
import proguard.classfile.util.SimplifiedVisitor;

public class ExceptClassConstantFilter
extends      SimplifiedVisitor
implements   ConstantVisitor
{
    private final String           exceptClassName;
    private final ConstantVisitor constantVisitor;


    /**
     * Creates a new ClassNameFilter.
     * @param exceptClassName the name of the class that will not be visited.
     * @param constantVisitor the <code>ConstantVisitor</code> to which visits
     *                        will be delegated.
     */
        public ExceptClassConstantFilter(String          exceptClassName,
                                         ConstantVisitor constantVisitor)
    {
            this.exceptClassName = exceptClassName;
            this.constantVisitor = constantVisitor;
    }


    // Implementations for ConstantVisitor.

    public void visitClassConstant(Clazz clazz, ClassConstant classConstant)
    {
        if (!classConstant.getName(clazz).equals(exceptClassName))
        {
            constantVisitor.visitClassConstant(clazz, classConstant);
        }
    }
}

package proguard.classfile.visitor;

import proguard.classfile.Clazz;
import proguard.classfile.util.SimplifiedVisitor;

import java.util.Set;

public class ClassCollector
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private final Set set;


    /**
     * Creates a new ClassCollector.
     * @param set the <code>Set</code> in which all class names will be
     *            collected.
     */
    public ClassCollector(Set set)
    {
        this.set = set;
    }


    // Implementations for ClassVisitor.

    public void visitAnyClass(Clazz clazz)
    {
        set.add(clazz);
    }
}


package proguard.preverify;

import proguard.Configuration;
import proguard.classfile.*;
import proguard.classfile.attribute.visitor.AllAttributeVisitor;
import proguard.classfile.visitor.*;

public class SubroutineInliner
{
    private final Configuration configuration;


    /**
     * Creates a new SubroutineInliner.
     */
    public SubroutineInliner(Configuration configuration)
    {
        this.configuration = configuration;
    }


    /**
     * Performs subroutine inlining of the given program class pool.
     */
    public void execute(ClassPool programClassPool)
    {
        // Clean up any old visitor info.
        programClassPool.classesAccept(new ClassCleaner());

        // Inline all subroutines.
        ClassVisitor inliner =
            new AllMethodVisitor(
            new AllAttributeVisitor(
            new CodeSubroutineInliner()));

        // In Java Standard Edition, only class files from Java 6 or higher
        // should be preverified.
        if (!configuration.microEdition)
        {
            inliner =
                new ClassVersionFilter(ClassConstants.INTERNAL_CLASS_VERSION_1_6,
                                       Integer.MAX_VALUE,
                                       inliner);
        }

        programClassPool.classesAccept(inliner);
    }
}

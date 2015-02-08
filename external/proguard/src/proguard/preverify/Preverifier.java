
package proguard.preverify;

import proguard.Configuration;
import proguard.classfile.*;
import proguard.classfile.attribute.visitor.AllAttributeVisitor;
import proguard.classfile.visitor.*;

public class Preverifier
{
    private final Configuration configuration;


    /**
     * Creates a new Preverifier.
     */
    public Preverifier(Configuration configuration)
    {
        this.configuration = configuration;
    }


    /**
     * Performs preverification of the given program class pool.
     */
    public void execute(ClassPool programClassPool)
    {
        // Clean up any old visitor info.
        programClassPool.classesAccept(new ClassCleaner());

        // Preverify all methods.
        ClassVisitor preverifier =
            new AllMethodVisitor(
            new AllAttributeVisitor(
            new CodePreverifier(configuration.microEdition)));

        // In Java Standard Edition, only class files from Java 6 or higher
        // should be preverified.
        if (!configuration.microEdition)
        {
            preverifier =
                new ClassVersionFilter(ClassConstants.INTERNAL_CLASS_VERSION_1_6,
                                       Integer.MAX_VALUE,
                                       preverifier);
        }

        programClassPool.classesAccept(preverifier);
    }
}

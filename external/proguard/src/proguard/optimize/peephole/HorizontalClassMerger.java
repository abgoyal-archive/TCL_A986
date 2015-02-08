
package proguard.optimize.peephole;

import proguard.classfile.*;
import proguard.classfile.util.SimplifiedVisitor;
import proguard.classfile.visitor.*;

public class HorizontalClassMerger
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private final boolean      allowAccessModification;
    private final boolean      mergeInterfacesAggressively;
    private final ClassVisitor extraClassVisitor;


    /**
     * Creates a new HorizontalClassMerger.
     * @param allowAccessModification     specifies whether the access modifiers
     *                                    of classes can be changed in order to
     *                                    merge them.
     * @param mergeInterfacesAggressively specifies whether interfaces may
     *                                    be merged aggressively.
     */
    public HorizontalClassMerger(boolean allowAccessModification,
                                 boolean mergeInterfacesAggressively)
    {
        this(allowAccessModification, mergeInterfacesAggressively, null);
    }


    /**
     * Creates a new VerticalClassMerger.
     * @param allowAccessModification     specifies whether the access modifiers
     *                                    of classes can be changed in order to
     *                                    merge them.
     * @param mergeInterfacesAggressively specifies whether interfaces may
     *                                    be merged aggressively.
     * @param extraClassVisitor           an optional extra visitor for all
     *                                    merged classes.
     */
    public HorizontalClassMerger(boolean      allowAccessModification,
                                 boolean      mergeInterfacesAggressively,
                                 ClassVisitor extraClassVisitor)
    {
        this.allowAccessModification     = allowAccessModification;
        this.mergeInterfacesAggressively = mergeInterfacesAggressively;
        this.extraClassVisitor           = extraClassVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        programClass.superClassConstantAccept(new ReferencedClassVisitor(
                                              new SubclassTraveler(
                                              new ProgramClassFilter(
                                              new ClassMerger(programClass,
                                                              allowAccessModification,
                                                              mergeInterfacesAggressively,
                                                              extraClassVisitor)))));
    }
}
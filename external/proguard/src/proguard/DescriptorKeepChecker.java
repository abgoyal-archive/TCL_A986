
package proguard;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;
import proguard.optimize.KeepMarker;

import java.util.List;


public class DescriptorKeepChecker
extends      SimplifiedVisitor
implements   MemberVisitor,
             ClassVisitor
{
    private final ClassPool      programClassPool;
    private final ClassPool      libraryClassPool;
    private final WarningPrinter notePrinter;

    // Some fields acting as parameters for the class visitor.
    private Clazz   referencingClass;
    private Member  referencingMember;
    private boolean isField;


    /**
     * Creates a new DescriptorKeepChecker.
     */
    public DescriptorKeepChecker(ClassPool      programClassPool,
                                 ClassPool      libraryClassPool,
                                 WarningPrinter notePrinter)
    {
        this.programClassPool = programClassPool;
        this.libraryClassPool = libraryClassPool;
        this.notePrinter      = notePrinter;
    }


    /**
     * Checks the classes mentioned in the given keep specifications, printing
     * notes if necessary. Returns the number of notes printed.
     */
    public void checkClassSpecifications(List keepSpecifications)
    {
        // Clean up any old visitor info.
        programClassPool.classesAccept(new ClassCleaner());
        libraryClassPool.classesAccept(new ClassCleaner());

        // Create a visitor for marking the seeds.
        KeepMarker keepMarker = new KeepMarker();
        ClassPoolVisitor classPoolvisitor =
            ClassSpecificationVisitorFactory.createClassPoolVisitor(keepSpecifications,
                                                                    keepMarker,
                                                                    keepMarker,
                                                                    false,
                                                                    true,
                                                                    true);
        // Mark the seeds.
        programClassPool.accept(classPoolvisitor);
        libraryClassPool.accept(classPoolvisitor);

        // Print out notes about argument types that are not being kept.
        programClassPool.classesAccept(new AllMemberVisitor(this));
    }


    // Implementations for MemberVisitor.

    public void visitProgramField(ProgramClass programClass, ProgramField programField)
    {
        if (KeepMarker.isKept(programField))
        {
            referencingClass  = programClass;
            referencingMember = programField;
            isField           = true;

            // Don't check the type, because it is not required for introspection.
            //programField.referencedClassesAccept(this);
        }
    }


    public void visitProgramMethod(ProgramClass programClass, ProgramMethod programMethod)
    {
        if (KeepMarker.isKept(programMethod))
        {
            referencingClass  = programClass;
            referencingMember = programMethod;
            isField           = false;

            // Don't check the return type, because it is not required for
            // introspection (e.g. the return type of the special Enum methods).
            //programMethod.referencedClassesAccept(this);

            Clazz[] referencedClasses = programMethod.referencedClasses;
            if (referencedClasses != null)
            {
                for (int index = 0; index < referencedClasses.length-1; index++)
                {
                    if (referencedClasses[index] != null)
                    {
                        referencedClasses[index].accept(this);
                    }
                }
            }
        }
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (!KeepMarker.isKept(programClass))
        {
            notePrinter.print(referencingClass.getName(),
                              programClass.getName(),
                              "Note: the configuration keeps the entry point '" +
                              ClassUtil.externalClassName(referencingClass.getName()) +
                              " { " +
                              (isField ?
                                   ClassUtil.externalFullFieldDescription(0,
                                                                          referencingMember.getName(referencingClass),
                                                                          referencingMember.getDescriptor(referencingClass)) :
                                   ClassUtil.externalFullMethodDescription(referencingClass.getName(),
                                                                           0,
                                                                           referencingMember.getName(referencingClass),
                                                                           referencingMember.getDescriptor(referencingClass))) +
                              "; }', but not the descriptor class '" +
                              ClassUtil.externalClassName(programClass.getName()) +
                              "'");
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass) {}
}

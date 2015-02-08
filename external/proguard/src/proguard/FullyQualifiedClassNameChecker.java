
package proguard;

import proguard.classfile.*;
import proguard.classfile.util.*;
import proguard.classfile.visitor.*;

import java.util.List;

public class FullyQualifiedClassNameChecker
extends      SimplifiedVisitor
implements   ClassVisitor
{
    private final ClassPool      programClassPool;
    private final ClassPool      libraryClassPool;
    private final WarningPrinter notePrinter;


    /**
     * Creates a new DescriptorKeepChecker.
     */
    public FullyQualifiedClassNameChecker(ClassPool      programClassPool,
                                          ClassPool      libraryClassPool,
                                          WarningPrinter notePrinter)
    {
        this.programClassPool = programClassPool;
        this.libraryClassPool = libraryClassPool;
        this.notePrinter      = notePrinter;
    }


    /**
     * Checks the classes mentioned in the given class specifications, printing
     * notes if necessary. Returns the number of notes printed.
     */
    public void checkClassSpecifications(List classSpecifications)
    {
        if (classSpecifications != null)
        {
            for (int index = 0; index < classSpecifications.size(); index++)
            {
                ClassSpecification classSpecification =
                    (ClassSpecification)classSpecifications.get(index);

                checkType(classSpecification.annotationType);
                checkClassName(classSpecification.className);
                checkType(classSpecification.extendsAnnotationType);
                checkClassName(classSpecification.extendsClassName);

                checkMemberSpecifications(classSpecification.fieldSpecifications,  true);
                checkMemberSpecifications(classSpecification.methodSpecifications, false);
            }
        }
    }


    /**
     * Checks the classes mentioned in the given class member specifications,
     * printing notes if necessary.
     */
    private void checkMemberSpecifications(List memberSpecifications, boolean isField)
    {
        if (memberSpecifications != null)
        {
            for (int index = 0; index < memberSpecifications.size(); index++)
            {
                MemberSpecification memberSpecification =
                    (MemberSpecification)memberSpecifications.get(index);

                checkType(memberSpecification.annotationType);

                if (isField)
                {
                     checkType(memberSpecification.descriptor);
                }
                else
                {
                    checkDescriptor(memberSpecification.descriptor);
                }
            }
        }
    }


    /**
     * Checks the classes mentioned in the given class member descriptor,
     * printing notes if necessary.
     */
    private void checkDescriptor(String descriptor)
    {
        if (descriptor != null)
        {
            InternalTypeEnumeration internalTypeEnumeration =
                new InternalTypeEnumeration(descriptor);

            checkType(internalTypeEnumeration.returnType());

            while (internalTypeEnumeration.hasMoreTypes())
            {
                checkType(internalTypeEnumeration.nextType());
            }
        }
    }


    /**
     * Checks the class mentioned in the given type (if any),
     * printing notes if necessary.
     */
    private void checkType(String type)
    {
        if (type != null)
        {
            checkClassName(ClassUtil.internalClassNameFromType(type));
        }
    }


    /**
     * Checks the specified class (if any),
     * printing notes if necessary.
     */
    private void checkClassName(String className)
    {
        if (className != null                            &&
            !containsWildCards(className)                &&
            programClassPool.getClass(className) == null &&
            libraryClassPool.getClass(className) == null &&
            notePrinter.accepts(className))
        {
            notePrinter.print(className,
                              "Note: the configuration refers to the unknown class '" +
                              ClassUtil.externalClassName(className) + "'");

            String fullyQualifiedClassName =
                "**" + ClassConstants.INTERNAL_PACKAGE_SEPARATOR +
                className.substring(className.lastIndexOf(ClassConstants.INTERNAL_PACKAGE_SEPARATOR)+1);

            ClassNameFilter classNameFilter =
                new ClassNameFilter(fullyQualifiedClassName, this);

            programClassPool.classesAccept(classNameFilter);
            libraryClassPool.classesAccept(classNameFilter);
        }
    }


    private static boolean containsWildCards(String string)
    {
        return string != null &&
            (string.indexOf('*')   >= 0 ||
             string.indexOf('?')   >= 0 ||
             string.indexOf(',')   >= 0 ||
             string.indexOf("///") >= 0);
    }


    // Implementations for ClassVisitor.

    public void visitAnyClass(Clazz clazz)
    {
        System.out.println("      Maybe you meant the fully qualified name '" +
                           ClassUtil.externalClassName(clazz.getName()) + "'?");
    }
}

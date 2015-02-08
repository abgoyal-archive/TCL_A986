
package proguard.classfile.util;

import proguard.classfile.*;
import proguard.classfile.visitor.*;

public class MemberFinder
extends      SimplifiedVisitor
implements   MemberVisitor
{
    private static class MemberFoundException extends RuntimeException {}
    private static final MemberFoundException MEMBER_FOUND = new MemberFoundException();

    private Clazz  clazz;
    private Member member;


    /**
     * Finds the field with the given name and descriptor in the given
     * class or its hierarchy.
     */
    public Field findField(Clazz  referencingClass,
                           Clazz  clazz,
                           String name,
                           String descriptor)
    {
        return (Field)findMember(referencingClass, clazz, name, descriptor, true);
    }


    /**
     * Finds the method with the given name and descriptor in the given
     * class or its hierarchy.
     */
    public Method findMethod(Clazz  referencingClass,
                             Clazz  clazz,
                             String name,
                             String descriptor)
    {
        return (Method)findMember(referencingClass, clazz, name, descriptor, false);
    }


    /**
     * Finds the class member with the given name and descriptor in the given
     * class or its hierarchy.
     */
    public Member findMember(Clazz   referencingClass,
                             Clazz   clazz,
                             String  name,
                             String  descriptor,
                             boolean isField)
    {
        // Organize a search in the hierarchy of superclasses and interfaces.
        // The class member may be in a different class, if the code was
        // compiled with "-target 1.2" or higher (the default in JDK 1.4).
        try
        {
            this.clazz  = null;
            this.member = null;
            clazz.hierarchyAccept(true, true, true, false, isField ?
                (ClassVisitor)new NamedFieldVisitor(name, descriptor,
                              new MemberClassAccessFilter(referencingClass, this)) :
                (ClassVisitor)new NamedMethodVisitor(name, descriptor,
                              new MemberClassAccessFilter(referencingClass, this)));
        }
        catch (MemberFoundException ex)
        {
            // We've found the member we were looking for.
        }

        return member;
    }


    /**
     * Returns the corresponding class of the most recently found class
     * member.
     */
    public Clazz correspondingClass()
    {
        return clazz;
    }


    /**
     * Returns whether the given method is overridden anywhere down the class
     * hierarchy.
     */
    public boolean isOverriden(Clazz  clazz,
                               Method method)
    {
        String name       = method.getName(clazz);
        String descriptor = method.getDescriptor(clazz);

        // Go looking for the method down the class hierarchy.
        try
        {
            this.clazz  = null;
            this.member = null;

            clazz.hierarchyAccept(false, false, false, true,
                new NamedMethodVisitor(name, descriptor,
                new MemberAccessFilter(0, ClassConstants.INTERNAL_ACC_PRIVATE, this)));
        }
        catch (MemberFoundException ex)
        {
            // We've found an overriding method.
            return true;
        }

        return false;
    }


    /**
     * Returns whether the given field is shadowed anywhere down the class
     * hierarchy.
     */
    public boolean isShadowed(Clazz clazz,
                              Field field)
    {
        String name       = field.getName(clazz);
        String descriptor = field.getDescriptor(clazz);

        // Go looking for the field down the class hierarchy.
        try
        {
            this.clazz  = null;
            this.member = null;
            clazz.hierarchyAccept(false, false, false, true,
                new NamedFieldVisitor(name, descriptor,
                new MemberAccessFilter(0, ClassConstants.INTERNAL_ACC_PRIVATE, this)));
        }
        catch (MemberFoundException ex)
        {
            // We've found a shadowing field.
            return true;
        }

        return false;
    }


//    // Implementations for ClassVisitor.
//
//    private void visitAnyClass(Clazz clazz)
//    {
//        if (member == null)
//        {
//            member = isField ?
//                (Member)clazz.findField(name, descriptor) :
//                (Member)clazz.findMethod(name, descriptor);
//
//            if (member != null)
//            {
//                this.clazz = clazz;
//            }
//        }
//    }


    // Implementations for MemberVisitor.

    public void visitAnyMember(Clazz clazz, Member member)
    {
        this.clazz  = clazz;
        this.member = member;

        throw MEMBER_FOUND;
    }
}

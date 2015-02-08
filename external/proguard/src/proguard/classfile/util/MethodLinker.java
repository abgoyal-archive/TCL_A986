
package proguard.classfile.util;

import proguard.classfile.*;
import proguard.classfile.visitor.*;

import java.util.*;

public class MethodLinker
extends      SimplifiedVisitor
implements   ClassVisitor,
             MemberVisitor
{
    // An object that is reset and reused every time.
    // The map: [class member name+' '+descriptor - class member info]
    private final Map memberMap = new HashMap();


    // Implementations for ClassVisitor.

    public void visitAnyClass(Clazz clazz)
    {
        // Collect all non-private members in this class hierarchy.
        clazz.hierarchyAccept(true, true, true, false,
            new AllMethodVisitor(
            new MemberAccessFilter(0, ClassConstants.INTERNAL_ACC_PRIVATE,
            this)));

        // Clean up for the next class hierarchy.
        memberMap.clear();
    }


    // Implementations for MemberVisitor.

    public void visitAnyMember(Clazz clazz, Member member)
    {
        // Get the class member's name and descriptor.
        String name       = member.getName(clazz);
        String descriptor = member.getDescriptor(clazz);

        // Special cases: <clinit> and <init> are always kept unchanged.
        // We can ignore them here.
        if (name.equals(ClassConstants.INTERNAL_METHOD_NAME_CLINIT) ||
            name.equals(ClassConstants.INTERNAL_METHOD_NAME_INIT))
        {
            return;
        }

        // See if we've already come across a method with the same name and
        // descriptor.
        String key = name + ' ' + descriptor;
        Member otherMember = (Member)memberMap.get(key);

        if (otherMember == null)
        {
            // Get the last method in the chain.
            Member thisLastMember = lastMember(member);

            // Store the new class method in the map.
            memberMap.put(key, thisLastMember);
        }
        else
        {
            // Link both members.
            link(member, otherMember);
        }
    }


    // Small utility methods.

    /**
     * Links the two given class members.
     */
    private static void link(Member member1, Member member2)
    {
        // Get the last methods in the both chains.
        Member lastMember1 = lastMember(member1);
        Member lastMember2 = lastMember(member2);

        // Check if both link chains aren't already ending in the same element.
        if (!lastMember1.equals(lastMember2))
        {
            // Merge the two chains, with the library members last.
            if (lastMember2 instanceof LibraryMember)
            {
                lastMember1.setVisitorInfo(lastMember2);
            }
            else
            {
                lastMember2.setVisitorInfo(lastMember1);
            }
        }
    }


    /**
     * Finds the last class member in the linked list of related class members.
     * @param member the given class member.
     * @return the last class member in the linked list.
     */
    public static Member lastMember(Member member)
    {
        Member lastMember = member;
        while (lastMember.getVisitorInfo() != null &&
               lastMember.getVisitorInfo() instanceof Member)
        {
            lastMember = (Member)lastMember.getVisitorInfo();
        }

        return lastMember;
    }


    /**
     * Finds the last visitor accepter in the linked list of visitors.
     * @param visitorAccepter the given method.
     * @return the last method in the linked list.
     */
    public static VisitorAccepter lastVisitorAccepter(VisitorAccepter visitorAccepter)
    {
        VisitorAccepter lastVisitorAccepter = visitorAccepter;
        while (lastVisitorAccepter.getVisitorInfo() != null &&
               lastVisitorAccepter.getVisitorInfo() instanceof VisitorAccepter)
        {
            lastVisitorAccepter = (VisitorAccepter)lastVisitorAccepter.getVisitorInfo();
        }

        return lastVisitorAccepter;
    }
}

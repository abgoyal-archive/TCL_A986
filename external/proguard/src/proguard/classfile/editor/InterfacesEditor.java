
package proguard.classfile.editor;

import proguard.classfile.*;
import proguard.classfile.attribute.*;

import java.util.Arrays;

public class InterfacesEditor
{
    private final ProgramClass targetClass;


    /**
     * Creates a new InterfacesEditor that will edit interfaces in the given
     * target class.
     */
    public InterfacesEditor(ProgramClass targetClass)
    {
        this.targetClass = targetClass;
    }


    /**
     * Adds the specified interface to the target class, if it isn't present yet.
     */
    public void addInterface(int interfaceConstantIndex)
    {
        // Is the interface not yet present?
        if (findInterfaceIndex(interfaceConstantIndex) < 0)
        {
            int   interfacesCount = targetClass.u2interfacesCount++;
            int[] interfaces      = targetClass.u2interfaces;

            // Is the array too small to contain the additional interface?
            if (interfaces.length <= interfacesCount)
            {
                // Create a new array and copy the interfaces into it.
                int[] newinterfaces = new int[interfacesCount + 1];
                System.arraycopy(interfaces, 0, newinterfaces, 0, interfacesCount);
                interfaces = newinterfaces;

                targetClass.u2interfaces = interfaces;
            }

            // Append the interface.
            interfaces[interfacesCount] = interfaceConstantIndex;
        }
    }


    /**
     * Deletes the given interface from the target class, if it is present.
     */
    public void deleteInterface(int interfaceConstantIndex)
    {
        // Is the interface already present?
        int interfaceIndex = findInterfaceIndex(interfaceConstantIndex);
        if (interfaceIndex >= 0)
        {
            int   interfacesCount = --targetClass.u2interfacesCount;
            int[] interfaces      = targetClass.u2interfaces;

            // Shift the other interfaces in the array.
            for (int index = interfaceIndex; index < interfacesCount; index++)
            {
                interfaces[index] = interfaces[index + 1];
            }

            // Clear the remaining entry in the array.
            interfaces[interfacesCount] = 0;
        }
    }


    // Small utility methods.

    /**
     * Finds the index of the specified interface in the list of interfaces of
     * the target class.
     */
    private int findInterfaceIndex(int interfaceConstantIndex)
    {
        int   interfacesCount = targetClass.u2interfacesCount;
        int[] interfaces      = targetClass.u2interfaces;

        for (int index = 0; index < interfacesCount; index++)
        {
            if (interfaces[index] == interfaceConstantIndex)
              {
                  return index;
              }
        }

        return -1;
    }
}
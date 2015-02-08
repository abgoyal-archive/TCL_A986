
package proguard.obfuscate;

import java.util.*;


public class NumericNameFactory implements NameFactory
{
    private int index;


    // Implementations for NameFactory.

    public void reset()
    {
        index = 0;
    }


    public String nextName()
    {
        return Integer.toString(++index);
    }
}
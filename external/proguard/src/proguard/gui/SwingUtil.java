
package proguard.gui;

import javax.swing.*;
import java.lang.reflect.InvocationTargetException;


public class SwingUtil
{
    /**
     * Invokes the given Runnable in the AWT event dispatching thread,
     * and waits for it to finish. This method may be called from any thread,
     * including the event dispatching thread itself.
     * @see SwingUtilities#invokeAndWait(Runnable)
     * @param runnable the Runnable to be executed.
     */
    public static void invokeAndWait(Runnable runnable)
    throws InterruptedException, InvocationTargetException
    {
        try
        {
            if (SwingUtilities.isEventDispatchThread())
            {
                runnable.run();
            }
            else
            {
                SwingUtilities.invokeAndWait(runnable);
            }
        }
        catch (Exception ex)
        {
            // Ignore any exceptions.
        }
    }


    /**
     * Invokes the given Runnable in the AWT event dispatching thread, not
     * necessarily right away. This method may be called from any thread,
     * including the event dispatching thread itself.
     * @see SwingUtilities#invokeLater(Runnable)
     * @param runnable the Runnable to be executed.
     */
    public static void invokeLater(Runnable runnable)
    {
        if (SwingUtilities.isEventDispatchThread())
        {
            runnable.run();
        }
        else
        {
            SwingUtilities.invokeLater(runnable);
        }
    }
}

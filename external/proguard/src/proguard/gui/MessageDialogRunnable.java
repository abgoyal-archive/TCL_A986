
package proguard.gui;

import javax.swing.*;
import java.awt.*;
import java.lang.reflect.InvocationTargetException;


final class MessageDialogRunnable implements Runnable
{
    private final Component parentComponent;
    private final Object    message;
    private final String    title;
    private final int       messageType;


    /**
     * Creates a new MessageDialogRunnable object.
     * @see JOptionPane#showMessageDialog(Component, Object, String, int)
     */
    public static void showMessageDialog(Component parentComponent,
                                         Object    message,
                                         String    title,
                                         int       messageType)
    {
        try
        {
            SwingUtil.invokeAndWait(new MessageDialogRunnable(parentComponent,
                                                              message,
                                                              title,
                                                              messageType));
        }
        catch (Exception e)
        {
            // Nothing.
        }
    }


    /**
     * Creates a new MessageDialogRunnable object.
     * @see JOptionPane#showMessageDialog(Component, Object, String, int)
     */
    public MessageDialogRunnable(Component parentComponent,
                                 Object    message,
                                 String    title,
                                 int       messageType)
    {
        this.parentComponent = parentComponent;
        this.message         = message;
        this.title           = title;
        this.messageType     = messageType;
    }



    // Implementation for Runnable.

    public void run()
    {
        JOptionPane.showMessageDialog(parentComponent,
                                      message,
                                      title,
                                      messageType);
    }
}

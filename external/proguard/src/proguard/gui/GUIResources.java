
package proguard.gui;

import java.text.MessageFormat;
import java.util.ResourceBundle;


class GUIResources
{
    private static final ResourceBundle messages  = ResourceBundle.getBundle(GUIResources.class.getName());
    private static final MessageFormat  formatter = new MessageFormat("");


    /**
     * Returns an internationalized message, based on its key.
     */
    public static String getMessage(String messageKey)
    {
        return messages.getString(messageKey);
    }


    /**
     * Returns an internationalized, formatted message, based on its key, with
     * the given arguments.
     */
    public static String getMessage(String messageKey, Object[] messageArguments)
    {
        formatter.applyPattern(messages.getString(messageKey));
        return formatter.format(messageArguments);
    }
}

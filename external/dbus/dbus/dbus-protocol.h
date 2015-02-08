
/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef DBUS_PROTOCOL_H
#define DBUS_PROTOCOL_H


#ifdef  __cplusplus
extern "C" {
#if 0
} /* avoids confusing emacs indentation */
#endif
#endif

/* Normally docs are in .c files, but there isn't a .c file for this. */


/* Message byte order */
#define DBUS_LITTLE_ENDIAN ('l')  /**< Code marking LSB-first byte order in the wire protocol. */
#define DBUS_BIG_ENDIAN    ('B')  /**< Code marking MSB-first byte order in the wire protocol. */

/** Protocol version. */
#define DBUS_MAJOR_PROTOCOL_VERSION 1

/** Type code that is never equal to a legitimate type code */
#define DBUS_TYPE_INVALID       ((int) '\0')
/** #DBUS_TYPE_INVALID as a string literal instead of a int literal */
#define DBUS_TYPE_INVALID_AS_STRING        "\0"

/* Primitive types */
/** Type code marking an 8-bit unsigned integer */
#define DBUS_TYPE_BYTE          ((int) 'y')
/** #DBUS_TYPE_BYTE as a string literal instead of a int literal */
#define DBUS_TYPE_BYTE_AS_STRING           "y"
/** Type code marking a boolean */
#define DBUS_TYPE_BOOLEAN       ((int) 'b')
/** #DBUS_TYPE_BOOLEAN as a string literal instead of a int literal */
#define DBUS_TYPE_BOOLEAN_AS_STRING        "b"
/** Type code marking a 16-bit signed integer */
#define DBUS_TYPE_INT16         ((int) 'n')
/** #DBUS_TYPE_INT16 as a string literal instead of a int literal */
#define DBUS_TYPE_INT16_AS_STRING          "n"
/** Type code marking a 16-bit unsigned integer */
#define DBUS_TYPE_UINT16        ((int) 'q')
/** #DBUS_TYPE_UINT16 as a string literal instead of a int literal */
#define DBUS_TYPE_UINT16_AS_STRING         "q"
/** Type code marking a 32-bit signed integer */
#define DBUS_TYPE_INT32         ((int) 'i')
/** #DBUS_TYPE_INT32 as a string literal instead of a int literal */
#define DBUS_TYPE_INT32_AS_STRING          "i"
/** Type code marking a 32-bit unsigned integer */
#define DBUS_TYPE_UINT32        ((int) 'u')
/** #DBUS_TYPE_UINT32 as a string literal instead of a int literal */
#define DBUS_TYPE_UINT32_AS_STRING         "u"
/** Type code marking a 64-bit signed integer */
#define DBUS_TYPE_INT64         ((int) 'x')
/** #DBUS_TYPE_INT64 as a string literal instead of a int literal */
#define DBUS_TYPE_INT64_AS_STRING          "x"
/** Type code marking a 64-bit unsigned integer */
#define DBUS_TYPE_UINT64        ((int) 't')
/** #DBUS_TYPE_UINT64 as a string literal instead of a int literal */
#define DBUS_TYPE_UINT64_AS_STRING         "t"
/** Type code marking an 8-byte double in IEEE 754 format */
#define DBUS_TYPE_DOUBLE        ((int) 'd')
/** #DBUS_TYPE_DOUBLE as a string literal instead of a int literal */
#define DBUS_TYPE_DOUBLE_AS_STRING         "d"
/** Type code marking a UTF-8 encoded, nul-terminated Unicode string */
#define DBUS_TYPE_STRING        ((int) 's')
/** #DBUS_TYPE_STRING as a string literal instead of a int literal */
#define DBUS_TYPE_STRING_AS_STRING         "s"
/** Type code marking a D-Bus object path */
#define DBUS_TYPE_OBJECT_PATH   ((int) 'o')
/** #DBUS_TYPE_OBJECT_PATH as a string literal instead of a int literal */
#define DBUS_TYPE_OBJECT_PATH_AS_STRING    "o"
/** Type code marking a D-Bus type signature */
#define DBUS_TYPE_SIGNATURE     ((int) 'g')
/** #DBUS_TYPE_SIGNATURE as a string literal instead of a int literal */
#define DBUS_TYPE_SIGNATURE_AS_STRING      "g"

/* Compound types */
/** Type code marking a D-Bus array type */
#define DBUS_TYPE_ARRAY         ((int) 'a')
/** #DBUS_TYPE_ARRAY as a string literal instead of a int literal */
#define DBUS_TYPE_ARRAY_AS_STRING          "a"
/** Type code marking a D-Bus variant type */
#define DBUS_TYPE_VARIANT       ((int) 'v')
/** #DBUS_TYPE_VARIANT as a string literal instead of a int literal */
#define DBUS_TYPE_VARIANT_AS_STRING        "v"

#define DBUS_TYPE_STRUCT        ((int) 'r')
/** #DBUS_TYPE_STRUCT as a string literal instead of a int literal */
#define DBUS_TYPE_STRUCT_AS_STRING         "r"
#define DBUS_TYPE_DICT_ENTRY    ((int) 'e')
/** #DBUS_TYPE_DICT_ENTRY as a string literal instead of a int literal */
#define DBUS_TYPE_DICT_ENTRY_AS_STRING     "e"

#define DBUS_NUMBER_OF_TYPES    (16)

/* characters other than typecodes that appear in type signatures */

/** Code marking the start of a struct type in a type signature */
#define DBUS_STRUCT_BEGIN_CHAR   ((int) '(')
/** #DBUS_STRUCT_BEGIN_CHAR as a string literal instead of a int literal */
#define DBUS_STRUCT_BEGIN_CHAR_AS_STRING   "("
/** Code marking the end of a struct type in a type signature */
#define DBUS_STRUCT_END_CHAR     ((int) ')')
/** #DBUS_STRUCT_END_CHAR a string literal instead of a int literal */
#define DBUS_STRUCT_END_CHAR_AS_STRING     ")"
/** Code marking the start of a dict entry type in a type signature */
#define DBUS_DICT_ENTRY_BEGIN_CHAR   ((int) '{')
/** #DBUS_DICT_ENTRY_BEGIN_CHAR as a string literal instead of a int literal */
#define DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING   "{"
/** Code marking the end of a dict entry type in a type signature */
#define DBUS_DICT_ENTRY_END_CHAR     ((int) '}')
/** #DBUS_DICT_ENTRY_END_CHAR as a string literal instead of a int literal */
#define DBUS_DICT_ENTRY_END_CHAR_AS_STRING     "}"

#define DBUS_MAXIMUM_NAME_LENGTH 255

/** This one is 255 so it fits in a byte */
#define DBUS_MAXIMUM_SIGNATURE_LENGTH 255

#define DBUS_MAXIMUM_MATCH_RULE_LENGTH 1024

#define DBUS_MAXIMUM_MATCH_RULE_ARG_NUMBER 63
  
#define DBUS_MAXIMUM_ARRAY_LENGTH (67108864)
/** Number of bits you need in an unsigned to store the max array size */
#define DBUS_MAXIMUM_ARRAY_LENGTH_BITS 26

#define DBUS_MAXIMUM_MESSAGE_LENGTH (DBUS_MAXIMUM_ARRAY_LENGTH * 2)
/** Number of bits you need in an unsigned to store the max message size */
#define DBUS_MAXIMUM_MESSAGE_LENGTH_BITS 27

#define DBUS_MAXIMUM_TYPE_RECURSION_DEPTH 32

/* Types of message */

/** This value is never a valid message type, see dbus_message_get_type() */
#define DBUS_MESSAGE_TYPE_INVALID       0
/** Message type of a method call message, see dbus_message_get_type() */
#define DBUS_MESSAGE_TYPE_METHOD_CALL   1
/** Message type of a method return message, see dbus_message_get_type() */
#define DBUS_MESSAGE_TYPE_METHOD_RETURN 2
/** Message type of an error reply message, see dbus_message_get_type() */
#define DBUS_MESSAGE_TYPE_ERROR         3
/** Message type of a signal message, see dbus_message_get_type() */
#define DBUS_MESSAGE_TYPE_SIGNAL        4

/* Header flags */

#define DBUS_HEADER_FLAG_NO_REPLY_EXPECTED 0x1
#define DBUS_HEADER_FLAG_NO_AUTO_START     0x2

/* Header fields */

/** Not equal to any valid header field code */
#define DBUS_HEADER_FIELD_INVALID        0
#define DBUS_HEADER_FIELD_PATH           1
#define DBUS_HEADER_FIELD_INTERFACE      2
/** Header field code for a member (method or signal). See dbus_message_set_member(). */
#define DBUS_HEADER_FIELD_MEMBER         3
#define DBUS_HEADER_FIELD_ERROR_NAME     4
#define DBUS_HEADER_FIELD_REPLY_SERIAL   5
#define DBUS_HEADER_FIELD_DESTINATION    6
#define DBUS_HEADER_FIELD_SENDER         7
#define DBUS_HEADER_FIELD_SIGNATURE      8

#define DBUS_HEADER_FIELD_LAST DBUS_HEADER_FIELD_SIGNATURE

#define DBUS_HEADER_SIGNATURE                   \
     DBUS_TYPE_BYTE_AS_STRING                   \
     DBUS_TYPE_BYTE_AS_STRING                   \
     DBUS_TYPE_BYTE_AS_STRING                   \
     DBUS_TYPE_BYTE_AS_STRING                   \
     DBUS_TYPE_UINT32_AS_STRING                 \
     DBUS_TYPE_UINT32_AS_STRING                 \
     DBUS_TYPE_ARRAY_AS_STRING                  \
     DBUS_STRUCT_BEGIN_CHAR_AS_STRING           \
     DBUS_TYPE_BYTE_AS_STRING                   \
     DBUS_TYPE_VARIANT_AS_STRING                \
     DBUS_STRUCT_END_CHAR_AS_STRING


#define DBUS_MINIMUM_HEADER_SIZE 16

/* Errors */
/** A generic error; "something went wrong" - see the error message for more. */
#define DBUS_ERROR_FAILED                     "org.freedesktop.DBus.Error.Failed"
/** There was not enough memory to complete an operation. */
#define DBUS_ERROR_NO_MEMORY                  "org.freedesktop.DBus.Error.NoMemory"
/** The bus doesn't know how to launch a service to supply the bus name you wanted. */
#define DBUS_ERROR_SERVICE_UNKNOWN            "org.freedesktop.DBus.Error.ServiceUnknown"
/** The bus name you referenced doesn't exist (i.e. no application owns it). */
#define DBUS_ERROR_NAME_HAS_NO_OWNER          "org.freedesktop.DBus.Error.NameHasNoOwner"
/** No reply to a message expecting one, usually means a timeout occurred. */
#define DBUS_ERROR_NO_REPLY                   "org.freedesktop.DBus.Error.NoReply"
/** Something went wrong reading or writing to a socket, for example. */
#define DBUS_ERROR_IO_ERROR                   "org.freedesktop.DBus.Error.IOError"
/** A D-Bus bus address was malformed. */
#define DBUS_ERROR_BAD_ADDRESS                "org.freedesktop.DBus.Error.BadAddress"
/** Requested operation isn't supported (like ENOSYS on UNIX). */
#define DBUS_ERROR_NOT_SUPPORTED              "org.freedesktop.DBus.Error.NotSupported"
/** Some limited resource is exhausted. */
#define DBUS_ERROR_LIMITS_EXCEEDED            "org.freedesktop.DBus.Error.LimitsExceeded"
/** Security restrictions don't allow doing what you're trying to do. */
#define DBUS_ERROR_ACCESS_DENIED              "org.freedesktop.DBus.Error.AccessDenied"
/** Authentication didn't work. */
#define DBUS_ERROR_AUTH_FAILED                "org.freedesktop.DBus.Error.AuthFailed"
/** Unable to connect to server (probably caused by ECONNREFUSED on a socket). */
#define DBUS_ERROR_NO_SERVER                  "org.freedesktop.DBus.Error.NoServer"
#define DBUS_ERROR_TIMEOUT                    "org.freedesktop.DBus.Error.Timeout"
/** No network access (probably ENETUNREACH on a socket). */
#define DBUS_ERROR_NO_NETWORK                 "org.freedesktop.DBus.Error.NoNetwork"
/** Can't bind a socket since its address is in use (i.e. EADDRINUSE). */
#define DBUS_ERROR_ADDRESS_IN_USE             "org.freedesktop.DBus.Error.AddressInUse"
/** The connection is disconnected and you're trying to use it. */
#define DBUS_ERROR_DISCONNECTED               "org.freedesktop.DBus.Error.Disconnected"
/** Invalid arguments passed to a method call. */
#define DBUS_ERROR_INVALID_ARGS               "org.freedesktop.DBus.Error.InvalidArgs"
/** Missing file. */
#define DBUS_ERROR_FILE_NOT_FOUND             "org.freedesktop.DBus.Error.FileNotFound"
/** Existing file and the operation you're using does not silently overwrite. */
#define DBUS_ERROR_FILE_EXISTS                "org.freedesktop.DBus.Error.FileExists"
/** Method name you invoked isn't known by the object you invoked it on. */
#define DBUS_ERROR_UNKNOWN_METHOD             "org.freedesktop.DBus.Error.UnknownMethod"
#define DBUS_ERROR_TIMED_OUT                  "org.freedesktop.DBus.Error.TimedOut"
/** Tried to remove or modify a match rule that didn't exist. */
#define DBUS_ERROR_MATCH_RULE_NOT_FOUND       "org.freedesktop.DBus.Error.MatchRuleNotFound"
/** The match rule isn't syntactically valid. */
#define DBUS_ERROR_MATCH_RULE_INVALID         "org.freedesktop.DBus.Error.MatchRuleInvalid"
/** While starting a new process, the exec() call failed. */
#define DBUS_ERROR_SPAWN_EXEC_FAILED          "org.freedesktop.DBus.Error.Spawn.ExecFailed"
/** While starting a new process, the fork() call failed. */
#define DBUS_ERROR_SPAWN_FORK_FAILED          "org.freedesktop.DBus.Error.Spawn.ForkFailed"
/** While starting a new process, the child exited with a status code. */
#define DBUS_ERROR_SPAWN_CHILD_EXITED         "org.freedesktop.DBus.Error.Spawn.ChildExited"
/** While starting a new process, the child exited on a signal. */
#define DBUS_ERROR_SPAWN_CHILD_SIGNALED       "org.freedesktop.DBus.Error.Spawn.ChildSignaled"
/** While starting a new process, something went wrong. */
#define DBUS_ERROR_SPAWN_FAILED               "org.freedesktop.DBus.Error.Spawn.Failed"
/** Tried to get a UNIX process ID and it wasn't available. */
#define DBUS_ERROR_UNIX_PROCESS_ID_UNKNOWN    "org.freedesktop.DBus.Error.UnixProcessIdUnknown"
/** A type signature is not valid. */
#define DBUS_ERROR_INVALID_SIGNATURE          "org.freedesktop.DBus.Error.InvalidSignature"
/** A file contains invalid syntax or is otherwise broken. */
#define DBUS_ERROR_INVALID_FILE_CONTENT       "org.freedesktop.DBus.Error.InvalidFileContent"
/** Asked for SELinux security context and it wasn't available. */
#define DBUS_ERROR_SELINUX_SECURITY_CONTEXT_UNKNOWN    "org.freedesktop.DBus.Error.SELinuxSecurityContextUnknown"

/* XML introspection format */

/** XML namespace of the introspection format version 1.0 */
#define DBUS_INTROSPECT_1_0_XML_NAMESPACE         "http://www.freedesktop.org/standards/dbus"
/** XML public identifier of the introspection format version 1.0 */
#define DBUS_INTROSPECT_1_0_XML_PUBLIC_IDENTIFIER "-//freedesktop//DTD D-BUS Object Introspection 1.0//EN"
/** XML system identifier of the introspection format version 1.0 */
#define DBUS_INTROSPECT_1_0_XML_SYSTEM_IDENTIFIER "http://www.freedesktop.org/standards/dbus/1.0/introspect.dtd"
/** XML document type declaration of the introspection format version 1.0 */
#define DBUS_INTROSPECT_1_0_XML_DOCTYPE_DECL_NODE "<!DOCTYPE node PUBLIC \""DBUS_INTROSPECT_1_0_XML_PUBLIC_IDENTIFIER"\"\n\""DBUS_INTROSPECT_1_0_XML_SYSTEM_IDENTIFIER"\">\n"

/** @} */

#ifdef __cplusplus
#if 0
{ /* avoids confusing emacs indentation */
#endif
}
#endif

#endif /* DBUS_PROTOCOL_H */

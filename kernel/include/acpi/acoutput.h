


#ifndef __ACOUTPUT_H__
#define __ACOUTPUT_H__


/* Component IDs are used in the global "DebugLayer" */

#define ACPI_UTILITIES              0x00000001
#define ACPI_HARDWARE               0x00000002
#define ACPI_EVENTS                 0x00000004
#define ACPI_TABLES                 0x00000008
#define ACPI_NAMESPACE              0x00000010
#define ACPI_PARSER                 0x00000020
#define ACPI_DISPATCHER             0x00000040
#define ACPI_EXECUTER               0x00000080
#define ACPI_RESOURCES              0x00000100
#define ACPI_CA_DEBUGGER            0x00000200
#define ACPI_OS_SERVICES            0x00000400
#define ACPI_CA_DISASSEMBLER        0x00000800

/* Component IDs for ACPI tools and utilities */

#define ACPI_COMPILER               0x00001000
#define ACPI_TOOLS                  0x00002000
#define ACPI_EXAMPLE                0x00004000
#define ACPI_DRIVER                 0x00008000

#define ACPI_ALL_COMPONENTS         0x0000FFFF
#define ACPI_COMPONENT_DEFAULT      (ACPI_ALL_COMPONENTS)

/* Component IDs reserved for ACPI drivers */

#define ACPI_ALL_DRIVERS            0xFFFF0000

#define ACPI_LV_INIT                0x00000001
#define ACPI_LV_DEBUG_OBJECT        0x00000002
#define ACPI_LV_INFO                0x00000004
#define ACPI_LV_REPAIR              0x00000008
#define ACPI_LV_ALL_EXCEPTIONS      0x0000000F

/* Trace verbosity level 1 [Standard Trace Level] */

#define ACPI_LV_INIT_NAMES          0x00000020
#define ACPI_LV_PARSE               0x00000040
#define ACPI_LV_LOAD                0x00000080
#define ACPI_LV_DISPATCH            0x00000100
#define ACPI_LV_EXEC                0x00000200
#define ACPI_LV_NAMES               0x00000400
#define ACPI_LV_OPREGION            0x00000800
#define ACPI_LV_BFIELD              0x00001000
#define ACPI_LV_TABLES              0x00002000
#define ACPI_LV_VALUES              0x00004000
#define ACPI_LV_OBJECTS             0x00008000
#define ACPI_LV_RESOURCES           0x00010000
#define ACPI_LV_USER_REQUESTS       0x00020000
#define ACPI_LV_PACKAGE             0x00040000
#define ACPI_LV_VERBOSITY1          0x0007FF40 | ACPI_LV_ALL_EXCEPTIONS

/* Trace verbosity level 2 [Function tracing and memory allocation] */

#define ACPI_LV_ALLOCATIONS         0x00100000
#define ACPI_LV_FUNCTIONS           0x00200000
#define ACPI_LV_OPTIMIZATIONS       0x00400000
#define ACPI_LV_VERBOSITY2          0x00700000 | ACPI_LV_VERBOSITY1
#define ACPI_LV_ALL                 ACPI_LV_VERBOSITY2

/* Trace verbosity level 3 [Threading, I/O, and Interrupts] */

#define ACPI_LV_MUTEX               0x01000000
#define ACPI_LV_THREADS             0x02000000
#define ACPI_LV_IO                  0x04000000
#define ACPI_LV_INTERRUPTS          0x08000000
#define ACPI_LV_VERBOSITY3          0x0F000000 | ACPI_LV_VERBOSITY2

/* Exceptionally verbose output -- also used in the global "DebugLevel"  */

#define ACPI_LV_AML_DISASSEMBLE     0x10000000
#define ACPI_LV_VERBOSE_INFO        0x20000000
#define ACPI_LV_FULL_TABLES         0x40000000
#define ACPI_LV_EVENTS              0x80000000
#define ACPI_LV_VERBOSE             0xF0000000

#define ACPI_DEBUG_LEVEL(dl)        (u32) dl,ACPI_DEBUG_PARAMETERS

#define ACPI_DB_INIT                ACPI_DEBUG_LEVEL (ACPI_LV_INIT)
#define ACPI_DB_DEBUG_OBJECT        ACPI_DEBUG_LEVEL (ACPI_LV_DEBUG_OBJECT)
#define ACPI_DB_INFO                ACPI_DEBUG_LEVEL (ACPI_LV_INFO)
#define ACPI_DB_REPAIR              ACPI_DEBUG_LEVEL (ACPI_LV_REPAIR)
#define ACPI_DB_ALL_EXCEPTIONS      ACPI_DEBUG_LEVEL (ACPI_LV_ALL_EXCEPTIONS)

/* Trace level -- also used in the global "DebugLevel" */

#define ACPI_DB_INIT_NAMES          ACPI_DEBUG_LEVEL (ACPI_LV_INIT_NAMES)
#define ACPI_DB_THREADS             ACPI_DEBUG_LEVEL (ACPI_LV_THREADS)
#define ACPI_DB_PARSE               ACPI_DEBUG_LEVEL (ACPI_LV_PARSE)
#define ACPI_DB_DISPATCH            ACPI_DEBUG_LEVEL (ACPI_LV_DISPATCH)
#define ACPI_DB_LOAD                ACPI_DEBUG_LEVEL (ACPI_LV_LOAD)
#define ACPI_DB_EXEC                ACPI_DEBUG_LEVEL (ACPI_LV_EXEC)
#define ACPI_DB_NAMES               ACPI_DEBUG_LEVEL (ACPI_LV_NAMES)
#define ACPI_DB_OPREGION            ACPI_DEBUG_LEVEL (ACPI_LV_OPREGION)
#define ACPI_DB_BFIELD              ACPI_DEBUG_LEVEL (ACPI_LV_BFIELD)
#define ACPI_DB_TABLES              ACPI_DEBUG_LEVEL (ACPI_LV_TABLES)
#define ACPI_DB_FUNCTIONS           ACPI_DEBUG_LEVEL (ACPI_LV_FUNCTIONS)
#define ACPI_DB_OPTIMIZATIONS       ACPI_DEBUG_LEVEL (ACPI_LV_OPTIMIZATIONS)
#define ACPI_DB_VALUES              ACPI_DEBUG_LEVEL (ACPI_LV_VALUES)
#define ACPI_DB_OBJECTS             ACPI_DEBUG_LEVEL (ACPI_LV_OBJECTS)
#define ACPI_DB_ALLOCATIONS         ACPI_DEBUG_LEVEL (ACPI_LV_ALLOCATIONS)
#define ACPI_DB_RESOURCES           ACPI_DEBUG_LEVEL (ACPI_LV_RESOURCES)
#define ACPI_DB_IO                  ACPI_DEBUG_LEVEL (ACPI_LV_IO)
#define ACPI_DB_INTERRUPTS          ACPI_DEBUG_LEVEL (ACPI_LV_INTERRUPTS)
#define ACPI_DB_USER_REQUESTS       ACPI_DEBUG_LEVEL (ACPI_LV_USER_REQUESTS)
#define ACPI_DB_PACKAGE             ACPI_DEBUG_LEVEL (ACPI_LV_PACKAGE)
#define ACPI_DB_MUTEX               ACPI_DEBUG_LEVEL (ACPI_LV_MUTEX)
#define ACPI_DB_EVENTS              ACPI_DEBUG_LEVEL (ACPI_LV_EVENTS)

#define ACPI_DB_ALL                 ACPI_DEBUG_LEVEL (ACPI_LV_ALL)

/* Defaults for debug_level, debug and normal */

#define ACPI_DEBUG_DEFAULT          (ACPI_LV_INFO | ACPI_LV_REPAIR)
#define ACPI_NORMAL_DEFAULT         (ACPI_LV_INIT | ACPI_LV_DEBUG_OBJECT | ACPI_LV_REPAIR)
#define ACPI_DEBUG_ALL              (ACPI_LV_AML_DISASSEMBLE | ACPI_LV_ALL_EXCEPTIONS | ACPI_LV_ALL)

#if defined (ACPI_DEBUG_OUTPUT) || !defined (ACPI_NO_ERROR_MESSAGES)
#define ACPI_MODULE_NAME(name)          static const char ACPI_UNUSED_VAR _acpi_module_name[] = name;
#else
#define ACPI_MODULE_NAME(name)
#endif

#ifndef ACPI_NO_ERROR_MESSAGES
#define AE_INFO                         _acpi_module_name, __LINE__

#define ACPI_INFO(plist)                acpi_info plist
#define ACPI_WARNING(plist)             acpi_warning plist
#define ACPI_EXCEPTION(plist)           acpi_exception plist
#define ACPI_ERROR(plist)               acpi_error plist
#define ACPI_DEBUG_OBJECT(obj,l,i)      acpi_ex_do_debug_object(obj,l,i)

#else

/* No error messages */

#define ACPI_INFO(plist)
#define ACPI_WARNING(plist)
#define ACPI_EXCEPTION(plist)
#define ACPI_ERROR(plist)
#define ACPI_DEBUG_OBJECT(obj,l,i)

#endif				/* ACPI_NO_ERROR_MESSAGES */

#ifdef ACPI_DEBUG_OUTPUT

#ifndef ACPI_GET_FUNCTION_NAME
#define ACPI_GET_FUNCTION_NAME          _acpi_function_name

#define ACPI_FUNCTION_NAME(name)        static const char _acpi_function_name[] = #name;

#else
/* Compiler supports __FUNCTION__ (or equivalent) -- Ignore this macro */

#define ACPI_FUNCTION_NAME(name)
#endif				/* ACPI_GET_FUNCTION_NAME */

#define ACPI_DEBUG_PARAMETERS           __LINE__, ACPI_GET_FUNCTION_NAME, _acpi_module_name, _COMPONENT

#define ACPI_DEBUG_PRINT(plist)         acpi_debug_print plist
#define ACPI_DEBUG_PRINT_RAW(plist)     acpi_debug_print_raw plist

#else
#define ACPI_FUNCTION_NAME(a)
#define ACPI_DEBUG_PRINT(pl)
#define ACPI_DEBUG_PRINT_RAW(pl)

#endif				/* ACPI_DEBUG_OUTPUT */

#endif				/* __ACOUTPUT_H__ */

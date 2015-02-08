
// cmd.h -- Command buffer and command execution

//===========================================================================



void Cbuf_Init (void);
// allocates an initial text buffer that will grow as needed

void Cbuf_AddText (const char *text);
// as new commands are generated from the console or keybindings,
// the text is added to the end of the command buffer.

void Cbuf_InsertText (const char *text);
// when a command wants to issue other commands immediately, the text is
// inserted at the beginning of the buffer, before any remaining unexecuted
// commands.

void Cbuf_Execute (void);
// Pulls off \n terminated lines of text from the command buffer and sends
// them through Cmd_ExecuteString.  Stops when the buffer is empty.
// Normally called once per frame, but may be explicitly invoked.
// Do not call inside a command function!

//===========================================================================


typedef void (*xcommand_t) (void);

typedef enum
{
    src_client,        // came in over a net connection as a clc_stringcmd
                    // host_client will be valid during this state.
    src_command,        // from the command buffer
    cmd_src_t_max = 1 << 30
} cmd_source_t;

extern    cmd_source_t    cmd_source;

void    Cmd_Init (void);

void    Cmd_AddCommand (const char *cmd_name, xcommand_t function);
// called by the init functions of other parts of the program to
// register commands and functions to call for them.
// The cmd_name is referenced later, so it should not be in temp memory

qboolean Cmd_Exists (const char *cmd_name);
// used by the cvar code to check for cvar / command name overlap

const char     *Cmd_CompleteCommand (const char *partial);
// attempts to match a partial command for automatic command line completion
// returns NULL if nothing fits

int        Cmd_Argc (void);
char    *Cmd_Argv (int arg);
char    *Cmd_Args (void);
// The functions that execute commands get their parameters with these
// functions. Cmd_Argv () will return an empty string, not a NULL
// if arg > argc, so string operations are allways safe.

int Cmd_CheckParm (const char *parm);
// Returns the position (1 to argc-1) in the command's argument list
// where the given parameter apears, or 0 if not present

void Cmd_TokenizeString (char *text);
// Takes a null terminated string.  Does not need to be /n terminated.
// breaks the string up into arg tokens.

void    Cmd_ExecuteString (char *text, cmd_source_t src);

// Same as Cmd_ExecuteString, but for string constants.
void    Cmd_ExecuteString2 (const char *text, cmd_source_t src);

// Parses a single line of text into arguments and tries to execute it.
// The text can come from the command buffer, a remote client, or stdin.

void    Cmd_ForwardToServer (void);
// adds the current command line as a clc_stringcmd to the client message.
// things like godmode, noclip, etc, are commands directed to the server,
// so when they are typed in at the console, they will need to be forwarded.

void    Cmd_Print (const char *text);
// used by command functions to send output to either the graphics console or
// passed as a print message to the client


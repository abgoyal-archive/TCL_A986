// cvar.h


typedef struct cvar_s
{
	char	*name;
	char	*string;
	qboolean archive;		// set to true to cause it to be saved to vars.rc
	qboolean server;		// notifies players when changed
	float	value;
	struct cvar_s *next;
} cvar_t;

// Helper macro to avoid missing initializer warnings 
#define CVAR2(NAME, STRING) {(char*) (NAME),(char*) (STRING), 0, 0, 0.0f, (struct cvar_s*) 0}
#define CVAR3(NAME, STRING, ARCHIVE) {(char*) (NAME), (char*) (STRING), (ARCHIVE), 0, 0.0f, (struct cvar_s*) 0}
#define CVAR4(NAME, STRING, ARCHIVE, INFO) {(char*) (NAME), (char*) (STRING), (ARCHIVE), (INFO), 0.0f, (struct cvar_s*) 0}

void 	Cvar_RegisterVariable (cvar_t *variable);
// registers a cvar that allready has the name, string, and optionally the
// archive elements set.

void 	Cvar_Set (const char *var_name, const char *value);
// equivelant to "<name> <variable>" typed at the console

void	Cvar_SetValue (const char *var_name, float value);
// expands value to a string and calls Cvar_Set

float	Cvar_VariableValue (const char *var_name);
// returns 0 if not defined or non numeric

const char	*Cvar_VariableString (const char *var_name);
// returns an empty string if not defined

const char 	*Cvar_CompleteVariable (const char *partial);
// attempts to match a partial variable name for command line completion
// returns NULL if nothing fits

qboolean Cvar_Command (void);
// called by Cmd_ExecuteString when Cmd_Argv(0) doesn't match a known
// command.  Returns true if the command was a variable reference that
// was handled. (print or change)

void 	Cvar_WriteVariables (FILE *f);
// Writes lines containing "set variable value" for all variables
// with the archive flag set to true.

cvar_t *Cvar_FindVar (const char *var_name);

extern cvar_t	*cvar_vars;

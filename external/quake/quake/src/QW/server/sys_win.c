#include <sys/types.h>
#include <sys/timeb.h>
#include "qwsvdef.h"
#include <winsock.h>
#include <conio.h>


cvar_t	sys_nostdout = {"sys_nostdout","0"};

int	Sys_FileTime (char *path)
{
	FILE	*f;
	
	f = fopen(path, "rb");
	if (f)
	{
		fclose(f);
		return 1;
	}
	
	return -1;
}

void Sys_mkdir (char *path)
{
	_mkdir(path);
}


void Sys_Error (char *error, ...)
{
	va_list		argptr;
	char		text[1024];

	va_start (argptr,error);
	vsprintf (text, error,argptr);
	va_end (argptr);

//    MessageBox(NULL, text, "Error", 0 /* MB_OK */ );
	printf ("ERROR: %s\n", text);

	exit (1);
}


double Sys_DoubleTime (void)
{
	double t;
    struct _timeb tstruct;
	static int	starttime;

	_ftime( &tstruct );
 
	if (!starttime)
		starttime = tstruct.time;
	t = (tstruct.time-starttime) + tstruct.millitm*0.001;
	
	return t;
}


char *Sys_ConsoleInput (void)
{
	static char	text[256];
	static int		len;
	int		c;

	// read a line out
	while (_kbhit())
	{
		c = _getch();
		putch (c);
		if (c == '\r')
		{
			text[len] = 0;
			putch ('\n');
			len = 0;
			return text;
		}
		if (c == 8)
		{
			if (len)
			{
				putch (' ');
				putch (c);
				len--;
				text[len] = 0;
			}
			continue;
		}
		text[len] = c;
		len++;
		text[len] = 0;
		if (len == sizeof(text))
			len = 0;
	}

	return NULL;
}


void Sys_Printf (char *fmt, ...)
{
	va_list		argptr;
	
	if (sys_nostdout.value)
		return;
		
	va_start (argptr,fmt);
	vprintf (fmt,argptr);
	va_end (argptr);
}

void Sys_Quit (void)
{
	exit (0);
}


void Sys_Init (void)
{
	Cvar_RegisterVariable (&sys_nostdout);
}

char	*newargv[256];

int main (int argc, char **argv)
{
	quakeparms_t	parms;
	double			newtime, time, oldtime;
	static	char	cwd[1024];
	struct timeval	timeout;
	fd_set			fdset;
	int				t;

	COM_InitArgv (argc, argv);
	
	parms.argc = com_argc;
	parms.argv = com_argv;

	parms.memsize = 16*1024*1024;

	if ((t = COM_CheckParm ("-heapsize")) != 0 &&
		t + 1 < com_argc)
		parms.memsize = Q_atoi (com_argv[t + 1]) * 1024;

	if ((t = COM_CheckParm ("-mem")) != 0 &&
		t + 1 < com_argc)
		parms.memsize = Q_atoi (com_argv[t + 1]) * 1024 * 1024;

	parms.membase = malloc (parms.memsize);

	if (!parms.membase)
		Sys_Error("Insufficient memory.\n");

	parms.basedir = ".";
	parms.cachedir = NULL;

	SV_Init (&parms);

// run one frame immediately for first heartbeat
	SV_Frame (0.1);		

//
// main loop
//
	oldtime = Sys_DoubleTime () - 0.1;
	while (1)
	{
	// select on the net socket and stdin
	// the only reason we have a timeout at all is so that if the last
	// connected client times out, the message would not otherwise
	// be printed until the next event.
		FD_ZERO(&fdset);
		FD_SET(net_socket, &fdset);
		timeout.tv_sec = 0;
		timeout.tv_usec = 100;
		if (select (net_socket+1, &fdset, NULL, NULL, &timeout) == -1)
			continue;

	// find time passed since last cycle
		newtime = Sys_DoubleTime ();
		time = newtime - oldtime;
		oldtime = newtime;
		
		SV_Frame (time);				
	}	

	return true;
}



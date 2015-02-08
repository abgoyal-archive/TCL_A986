

#include <includes.h>
#include <radiusclient.h>


static const char * months[] =
		{
			"Jan", "Feb", "Mar", "Apr", "May", "Jun",
			"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
		};

void rc_str2tm (char *valstr, struct tm *tm)
{
	int             i;

	/* Get the month */
	for (i = 0; i < 12; i++)
	{
		if (strncmp (months[i], valstr, 3) == 0)
		{
			tm->tm_mon = i;
			i = 13;
		}
	}

	/* Get the Day */
	tm->tm_mday = atoi (&valstr[4]);

	/* Now the year */
	tm->tm_year = atoi (&valstr[7]) - 1900;
}

void rc_mdelay(int msecs)
{
	struct timeval tv;

	tv.tv_sec = (int) msecs / 1000;
	tv.tv_usec = (msecs % 1000) * 1000;

	select(0,(fd_set *)NULL,(fd_set *)NULL,(fd_set *)NULL, &tv);
}


char *
rc_mksid (void)
{
  static char buf[15];
  static unsigned short int cnt = 0;
  sprintf (buf, "%08lX%04X%02hX",
	   (unsigned long int) time (NULL),
	   (unsigned int) getpid (),
	   cnt & 0xFF);
  cnt++;
  return buf;
}

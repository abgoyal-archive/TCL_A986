#include <stdlib.h>
#include <string.h>

#include <quakedef.h>

// char *date = "Oct 24 1996";
// char *time = "13:22:52";
char *buildDate = __DATE__ ;
char *buildTime = __TIME__ ;

char *mon[12] = 
{ "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
char mond[12] = 
{ 31,    28,    31,    30,    31,    30,    31,    31,    30,    31,    30,    31 };

// returns days since Oct 24 1996
int build_number( void )
{
	int m = 0; 
	int d = 0;
	int y = 0;
	int hr, min;
	static int b = 0;

	if (b != 0)
		return b;

	for (m = 0; m < 11; m++)
	{
		if (strncasecmp( &buildDate[0], mon[m], 3 ) == 0)
			break;
		d += mond[m];
	}

	d += atoi( &buildDate[4] ) - 1;

	y = atoi( &buildDate[7] ) - 1900;

	b = d + (int)((y - 1) * 365.25);

	if (((y % 4) == 0) && m > 1)
	{
		b += 1;
	}

	b -= 34995; // Oct 24 1996

	hr = (buildTime[0] - '0') * 10 + (buildTime[1] - '0');
	min = (buildTime[3] - '0') * 10 + (buildTime[4] - '0');
//	sec = (buildTime[6] - '0') * 10 + (buildTime[7] - '0');

	b *= 60*24;
	b += hr * 60 + min;

	return b;
}


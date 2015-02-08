

#include "iwlib.h"		/* Header */

/************************** DOCUMENTATION **************************/


/**************************** CONSTANTS ****************************/

static const char *	argtype[] = {
  "     ", "byte ", "char ", "", "int  ", "float", "addr " };

/************************* MISC SUBROUTINES **************************/

/*------------------------------------------------------------------*/
static void
iw_usage(void)
{
  fprintf(stderr, "Usage: iwpriv interface [private-command [private-arguments]]\n");
}

/************************* SETTING ROUTINES **************************/

/*------------------------------------------------------------------*/
static int
set_private_cmd(int		skfd,		/* Socket */
		char *		args[],		/* Command line args */
		int		count,		/* Args count */
		char *		ifname,		/* Dev name */
		char *		cmdname,	/* Command name */
		iwprivargs *	priv,		/* Private ioctl description */
		int		priv_num)	/* Number of descriptions */
{
  struct iwreq	wrq;
  u_char	buffer[4096];	/* Only that big in v25 and later */
  int		i = 0;		/* Start with first command arg */
  int		k;		/* Index in private description table */
  int		temp;
  int		subcmd = 0;	/* sub-ioctl index */
  int		offset = 0;	/* Space for sub-ioctl index */

  /* Check if we have a token index.
   * Do it now so that sub-ioctl takes precedence, and so that we
   * don't have to bother with it later on... */
  if((count >= 1) && (sscanf(args[0], "[%i]", &temp) == 1))
    {
      subcmd = temp;
      args++;
      count--;
    }

  /* Search the correct ioctl */
  k = -1;
  while((++k < priv_num) && strcmp(priv[k].name, cmdname));

  /* If not found... */
  if(k == priv_num)
    {
      fprintf(stderr, "Invalid command : %s\n", cmdname);
      return(-1);
    }
	  
  /* Watch out for sub-ioctls ! */
  if(priv[k].cmd < SIOCDEVPRIVATE)
    {
      int	j = -1;

      /* Find the matching *real* ioctl */
      while((++j < priv_num) && ((priv[j].name[0] != '\0') ||
				 (priv[j].set_args != priv[k].set_args) ||
				 (priv[j].get_args != priv[k].get_args)));

      /* If not found... */
      if(j == priv_num)
	{
	  fprintf(stderr, "Invalid private ioctl definition for : %s\n",
		  cmdname);
	  return(-1);
	}

      /* Save sub-ioctl number */
      subcmd = priv[k].cmd;
      /* Reserve one int (simplify alignment issues) */
      offset = sizeof(__u32);
      /* Use real ioctl definition from now on */
      k = j;

#if 0
      printf("<mapping sub-ioctl %s to cmd 0x%X-%d>\n", cmdname,
	     priv[k].cmd, subcmd);
#endif
    }

  /* If we have to set some data */
  if((priv[k].set_args & IW_PRIV_TYPE_MASK) &&
     (priv[k].set_args & IW_PRIV_SIZE_MASK))
    {
      switch(priv[k].set_args & IW_PRIV_TYPE_MASK)
	{
	case IW_PRIV_TYPE_BYTE:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    sscanf(args[i], "%i", &temp);
	    buffer[i] = (char) temp;
	  }
	  break;

	case IW_PRIV_TYPE_INT:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    sscanf(args[i], "%i", &temp);
	    ((__s32 *) buffer)[i] = (__s32) temp;
	  }
	  break;

	case IW_PRIV_TYPE_CHAR:
	  if(i < count)
	    {
	      /* Size of the string to fetch */
	      wrq.u.data.length = strlen(args[i]) + 1;
	      if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
		wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	      /* Fetch string */
	      memcpy(buffer, args[i], wrq.u.data.length);
	      buffer[sizeof(buffer) - 1] = '\0';
	      i++;
	    }
	  else
	    {
	      wrq.u.data.length = 1;
	      buffer[0] = '\0';
	    }
	  break;

	case IW_PRIV_TYPE_FLOAT:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    double		freq;
	    if(sscanf(args[i], "%lg", &(freq)) != 1)
	      {
		printf("Invalid float [%s]...\n", args[i]);
		return(-1);
	      }    
	    if(strchr(args[i], 'G')) freq *= GIGA;
	    if(strchr(args[i], 'M')) freq *= MEGA;
	    if(strchr(args[i], 'k')) freq *= KILO;
	    sscanf(args[i], "%i", &temp);
	    iw_float2freq(freq, ((struct iw_freq *) buffer) + i);
	  }
	  break;

	case IW_PRIV_TYPE_ADDR:
	  /* Number of args to fetch */
	  wrq.u.data.length = count;
	  if(wrq.u.data.length > (priv[k].set_args & IW_PRIV_SIZE_MASK))
	    wrq.u.data.length = priv[k].set_args & IW_PRIV_SIZE_MASK;

	  /* Fetch args */
	  for(; i < wrq.u.data.length; i++) {
	    if(iw_in_addr(skfd, ifname, args[i],
			  ((struct sockaddr *) buffer) + i) < 0)
	      {
		printf("Invalid address [%s]...\n", args[i]);
		return(-1);
	      }
	  }
	  break;

	default:
	  fprintf(stderr, "Not implemented...\n");
	  return(-1);
	}
	  
      if((priv[k].set_args & IW_PRIV_SIZE_FIXED) &&
	 (wrq.u.data.length != (priv[k].set_args & IW_PRIV_SIZE_MASK)))
	{
	  printf("The command %s needs exactly %d argument(s)...\n",
		 cmdname, priv[k].set_args & IW_PRIV_SIZE_MASK);
	  return(-1);
	}
    }	/* if args to set */
  else
    {
      wrq.u.data.length = 0L;
    }

  strncpy(wrq.ifr_name, ifname, IFNAMSIZ);

  /* Those two tests are important. They define how the driver
   * will have to handle the data */
  if((priv[k].set_args & IW_PRIV_SIZE_FIXED) &&
      ((iw_get_priv_size(priv[k].set_args) + offset) <= IFNAMSIZ))
    {
      /* First case : all SET args fit within wrq */
      if(offset)
	wrq.u.mode = subcmd;
      memcpy(wrq.u.name + offset, buffer, IFNAMSIZ - offset);
    }
  else
    {
      if((priv[k].set_args == 0) &&
	 (priv[k].get_args & IW_PRIV_SIZE_FIXED) &&
	 (iw_get_priv_size(priv[k].get_args) <= IFNAMSIZ))
	{
	  /* Second case : no SET args, GET args fit within wrq */
	  if(offset)
	    wrq.u.mode = subcmd;
	}
      else
	{
	  /* Third case : args won't fit in wrq, or variable number of args */
	  wrq.u.data.pointer = (caddr_t) buffer;
	  wrq.u.data.flags = subcmd;
	}
    }

  /* Perform the private ioctl */
  if(ioctl(skfd, priv[k].cmd, &wrq) < 0)
    {
      fprintf(stderr, "Interface doesn't accept private ioctl...\n");
      fprintf(stderr, "%s (%X): %s\n", cmdname, priv[k].cmd, strerror(errno));
      return(-1);
    }

  /* If we have to get some data */
  if((priv[k].get_args & IW_PRIV_TYPE_MASK) &&
     (priv[k].get_args & IW_PRIV_SIZE_MASK))
    {
      int	j;
      int	n = 0;		/* number of args */

      printf("%-8.16s  %s:", ifname, cmdname);

      /* Check where is the returned data */
      if((priv[k].get_args & IW_PRIV_SIZE_FIXED) &&
	 (iw_get_priv_size(priv[k].get_args) <= IFNAMSIZ))
	{
	  memcpy(buffer, wrq.u.name, IFNAMSIZ);
	  n = priv[k].get_args & IW_PRIV_SIZE_MASK;
	}
      else
	n = wrq.u.data.length;

      switch(priv[k].get_args & IW_PRIV_TYPE_MASK)
	{
	case IW_PRIV_TYPE_BYTE:
	  /* Display args */
	  for(j = 0; j < n; j++)
	    printf("%d  ", buffer[j]);
	  printf("\n");
	  break;

	case IW_PRIV_TYPE_INT:
	  /* Display args */
	  for(j = 0; j < n; j++)
	    printf("%d  ", ((__s32 *) buffer)[j]);
	  printf("\n");
	  break;

	case IW_PRIV_TYPE_CHAR:
	  /* Display args */
	  buffer[n] = '\0';
	  printf("%s\n", buffer);
	  break;

	case IW_PRIV_TYPE_FLOAT:
	  {
	    double		freq;
	    /* Display args */
	    for(j = 0; j < n; j++)
	      {
		freq = iw_freq2float(((struct iw_freq *) buffer) + j);
		if(freq >= GIGA)
		  printf("%gG  ", freq / GIGA);
		else
		  if(freq >= MEGA)
		  printf("%gM  ", freq / MEGA);
		else
		  printf("%gk  ", freq / KILO);
	      }
	    printf("\n");
	  }
	  break;

	case IW_PRIV_TYPE_ADDR:
	  {
	    char		scratch[128];
	    struct sockaddr *	hwa;
	    /* Display args */
	    for(j = 0; j < n; j++)
	      {
		hwa = ((struct sockaddr *) buffer) + j;
		if(j)
		  printf("           %.*s", 
			 (int) strlen(cmdname), "                ");
		printf("%s\n", iw_saether_ntop(hwa, scratch));
	      }
	  }
	  break;

	default:
	  fprintf(stderr, "Not yet implemented...\n");
	  return(-1);
	}
    }	/* if args to set */

  return(0);
}

/*------------------------------------------------------------------*/
static inline int
set_private(int		skfd,		/* Socket */
	    char *	args[],		/* Command line args */
	    int		count,		/* Args count */
	    char *	ifname)		/* Dev name */
{
  iwprivargs *	priv;
  int		number;		/* Max of private ioctl */
  int		ret;

  /* Read the private ioctls */
  number = iw_get_priv_info(skfd, ifname, &priv);

  /* Is there any ? */
  if(number <= 0)
    {
      /* Should I skip this message ? */
      fprintf(stderr, "%-8.16s  no private ioctls.\n\n",
	      ifname);
      if(priv)
	free(priv);
      return(-1);
    }

  /* Do it */
  ret = set_private_cmd(skfd, args + 1, count - 1, ifname, args[0],
			priv, number);

  free(priv);
  return(ret);
}

/************************ CATALOG FUNCTIONS ************************/

/*------------------------------------------------------------------*/
static int
print_priv_info(int		skfd,
		char *		ifname,
		char *		args[],
		int		count)
{
  int		k;
  iwprivargs *	priv;
  int		n;

  /* Avoid "Unused parameter" warning */
  args = args; count = count;

  /* Read the private ioctls */
  n = iw_get_priv_info(skfd, ifname, &priv);

  /* Is there any ? */
  if(n <= 0)
    {
      /* Should I skip this message ? */
      fprintf(stderr, "%-8.16s  no private ioctls.\n\n",
	      ifname);
    }
  else
    {
      printf("%-8.16s  Available private ioctls :\n", ifname);
      /* Print them all */
      for(k = 0; k < n; k++)
	if(priv[k].name[0] != '\0')
	  printf("          %-16.16s (%.4X) : set %3d %s & get %3d %s\n",
		 priv[k].name, priv[k].cmd,
		 priv[k].set_args & IW_PRIV_SIZE_MASK,
		 argtype[(priv[k].set_args & IW_PRIV_TYPE_MASK) >> 12],
		 priv[k].get_args & IW_PRIV_SIZE_MASK,
		 argtype[(priv[k].get_args & IW_PRIV_TYPE_MASK) >> 12]);
      printf("\n");
    }

  /* Cleanup */
  if(priv)
    free(priv);
  return(0);
}

/*------------------------------------------------------------------*/
static int
print_priv_all(int		skfd,
	       char *		ifname,
	       char *		args[],
	       int		count)
{
  int		k;
  iwprivargs *	priv;
  int		n;

  /* Avoid "Unused parameter" warning */
  args = args; count = count;

  /* Read the private ioctls */
  n = iw_get_priv_info(skfd, ifname, &priv);

  /* Is there any ? */
  if(n <= 0)
    {
      /* Should I skip this message ? */
      fprintf(stderr, "%-8.16s  no private ioctls.\n\n",
	      ifname);
    }
  else
    {
      printf("%-8.16s  Available read-only private ioctl :\n", ifname);
      /* Print them all */
      for(k = 0; k < n; k++)
	/* We call all ioctls that don't have a null name, don't require
	 * args and return some (avoid triggering "reset" commands) */
	if((priv[k].name[0] != '\0') && (priv[k].set_args == 0) &&
	   (priv[k].get_args != 0))
	  set_private_cmd(skfd, NULL, 0, ifname, priv[k].name,
			  priv, n);
      printf("\n");
    }

  /* Cleanup */
  if(priv)
    free(priv);
  return(0);
}

/********************** PRIVATE IOCTLS MANIPS ***********************/

#if 0
/*------------------------------------------------------------------*/
static int
set_roaming(int		skfd,		/* Socket */
	    char *	args[],		/* Command line args */
	    int		count,		/* Args count */
	    char *	ifname)		/* Dev name */
{
  u_char	buffer[1024];
  struct iwreq		wrq;
  int		i = 0;		/* Start with first arg */
  int		k;
  iwprivargs *	priv;
  int		number;
  int		roamcmd;
  char		RoamState;		/* buffer to hold new roam state */
  char		ChangeRoamState=0;	/* whether or not we are going to
					   change roam states */

  /* Read the private ioctls */
  number = iw_get_priv_info(skfd, ifname, &priv);

  /* Is there any ? */
  if(number <= 0)
    {
      /* Should I skip this message ? */
      fprintf(stderr, "%-8.16s  no private ioctls.\n\n",
	      ifname);
      if(priv)
	free(priv);
      return(-1);
    }

  /* Get the ioctl number */
  k = -1;
  while((++k < number) && strcmp(priv[k].name, "setroam"));
  if(k == number)
    {
      fprintf(stderr, "This device doesn't support roaming\n");
      free(priv);
      return(-1);
    }
  roamcmd = priv[k].cmd;

  /* Cleanup */
  free(priv);

  if(count != 1)
    {
      iw_usage();
      return(-1);
    }

  if(!strcasecmp(args[i], "on"))
    {
      printf("%-8.16s  enable roaming\n", ifname);
      if(!number)
	{
	  fprintf(stderr, "This device doesn't support roaming\n");
	  return(-1);
	}
      ChangeRoamState=1;
      RoamState=1;
    }
  else
    if(!strcasecmp(args[i], "off"))
      {
	i++;
	printf("%-8.16s  disable roaming\n",  ifname);
	if(!number)
	  {
	    fprintf(stderr, "This device doesn't support roaming\n");
	    return(-1);
	  }
	ChangeRoamState=1;
	RoamState=0;
      }
    else
      {
	iw_usage();
	return(-1);
      }

  if(ChangeRoamState)
    {
      strncpy(wrq.ifr_name, ifname, IFNAMSIZ);

      buffer[0]=RoamState;

      memcpy(wrq.u.name, &buffer, IFNAMSIZ);

      if(ioctl(skfd, roamcmd, &wrq) < 0)
	{
	  fprintf(stderr, "Roaming support is broken.\n");
	  return(-1);
	}
    }

  return(0);
}

/*------------------------------------------------------------------*/
static int
port_type(int		skfd,		/* Socket */
	  char *	args[],		/* Command line args */
	  int		count,		/* Args count */
	  char *	ifname)		/* Dev name */
{
  struct iwreq	wrq;
  int		i = 0;		/* Start with first arg */
  int		k;
  iwprivargs *	priv;
  int		number;
  char		ptype = 0;
  char *	modes[] = { "invalid", "managed (BSS)", "reserved", "ad-hoc" };

  /* Read the private ioctls */
  number = iw_get_priv_info(skfd, ifname, &priv);

  /* Is there any ? */
  if(number <= 0)
    {
      /* Should I skip this message ? */
      fprintf(stderr, "%-8.16s  no private ioctls.\n\n", ifname);
      if(priv)
	free(priv);
      return(-1);
    }

  /* Arguments ? */
  if(count == 0)
    {
      /* So, we just want to see the current value... */
      k = -1;
      while((++k < number) && strcmp(priv[k].name, "gport_type") &&
	     strcmp(priv[k].name, "get_port"));
      if(k == number)
	{
	  fprintf(stderr, "This device doesn't support getting port type\n");
	  goto err;
	}
      strncpy(wrq.ifr_name, ifname, IFNAMSIZ);

      /* Get it */
      if(ioctl(skfd, priv[k].cmd, &wrq) < 0)
	{
	  fprintf(stderr, "Port type support is broken.\n");
	  goto err;
	}
      ptype = *wrq.u.name;

      /* Display it */
      printf("%-8.16s  Current port mode is %s <port type is %d>.\n\n",
	     ifname, modes[(int) ptype], ptype);

      free(priv);
      return(0);
    }

  if(count != 1)
    {
      iw_usage();
      goto err;
    }

  /* Read it */
  /* As a string... */
  k = 0;
  while((k < 4) && strncasecmp(args[i], modes[k], 2))
    k++;
  if(k < 4)
    ptype = k;
  else
    /* ...or as an integer */
    if(sscanf(args[i], "%i", (int *) &ptype) != 1)
      {
	iw_usage();
	goto err;
      }
  
  k = -1;
  while((++k < number) && strcmp(priv[k].name, "sport_type") &&
	strcmp(priv[k].name, "set_port"));
  if(k == number)
    {
      fprintf(stderr, "This device doesn't support setting port type\n");
      goto err;
    }
  strncpy(wrq.ifr_name, ifname, IFNAMSIZ);

  *(wrq.u.name) = ptype;

  if(ioctl(skfd, priv[k].cmd, &wrq) < 0)
    {
      fprintf(stderr, "Invalid port type (or setting not allowed)\n");
      goto err;
    }

  free(priv);
  return(0);

 err:
  free(priv);
  return(-1);
}
#endif

/******************************* MAIN ********************************/

/*------------------------------------------------------------------*/
int
main(int	argc,
     char **	argv)
{
  int skfd;		/* generic raw socket desc.	*/
  int goterr = 0;

  /* Create a channel to the NET kernel. */
  if((skfd = iw_sockets_open()) < 0)
    {
      perror("socket");
      return(-1);
    }

  /* No argument : show the list of all devices + ioctl list */
  if(argc == 1)
    iw_enum_devices(skfd, &print_priv_info, NULL, 0);
  else
    /* Special cases take one... */
    /* All */
    if((!strncmp(argv[1], "-a", 2)) || (!strcmp(argv[1], "--all")))
      iw_enum_devices(skfd, &print_priv_all, NULL, 0);
    else
      /* Help */
      if((!strncmp(argv[1], "-h", 2)) || (!strcmp(argv[1], "--help")))
	iw_usage();
      else
	/* Version */
	if (!strcmp(argv[1], "-v") || !strcmp(argv[1], "--version"))
	  goterr = iw_print_version_info("iwpriv");
	else
	  /* The device name must be the first argument */
	  /* Name only : show for that device only */
	  if(argc == 2)
	    print_priv_info(skfd, argv[1], NULL, 0);
	  else
	    /* Special cases take two... */
	    /* All */
	    if((!strncmp(argv[2], "-a", 2)) ||
	       (!strcmp(argv[2], "--all")))
	      print_priv_all(skfd, argv[1], NULL, 0);
	    else
#if 0
	      /* Roaming */
	      if(!strncmp(argv[2], "roam", 4))
		goterr = set_roaming(skfd, argv + 3, argc - 3, argv[1]);
	      else
		/* Port type */
		if(!strncmp(argv[2], "port", 4))
		  goterr = port_type(skfd, argv + 3, argc - 3, argv[1]);
		else
#endif
		  /*-------------*/
		  /* Otherwise, it's a private ioctl */
		  goterr = set_private(skfd, argv + 2, argc - 2, argv[1]);

  /* Close the socket. */
  iw_sockets_close(skfd);

  return(goterr);
}

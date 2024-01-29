
/*
 * Copyright (C) 2012 by Darren Reed.
 *
 * See the IPFILTER.LICENCE file for details on licencing.
 *
 * $Id$
 */

#include <ctype.h>

#include "ipf.h"

int getproto(char *name);

int
getproto(char *name)
{
	struct protoent *p;
	char *s;

	for (s = name; *s != '\0'; s++)
		if (!ISDIGIT(*s))
			break;
	if (*s == '\0')
		return (atoi(name));

	if (!strcasecmp(name, "ip"))
		return (0);

	p = getprotobyname(name);
	if (p != NULL)
		return (p->p_proto);
	return (-1);
}

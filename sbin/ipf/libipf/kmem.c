
/*
 * Copyright (C) 2012 by Darren Reed.
 *
 * See the IPFILTER.LICENCE file for details on licencing.
 */
/*
 * kmemcpy() - copies n bytes from kernel memory into user buffer.
 * returns 0 on success, -1 on error.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/uio.h>

#include <net/if.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <kvm.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "kmem.h"

static kvm_t *kvm_f = NULL;

int
openkmem(char *kern, char *core)
{
	kvm_f = kvm_open(kern, core, NULL, O_RDONLY, NULL);
	if (kvm_f == NULL) {
		perror("openkmem:open");
		return (-1);
	}
	return (kvm_f != NULL);
}

int
kmemcpy(register char *buf, long pos, register int n)
{
	register int r;

	if (!n)
		return (0);

	if (kvm_f == NULL)
		if (openkmem(NULL, NULL) == -1)
			return (-1);

	while ((r = kvm_read(kvm_f, pos, buf, n)) < n)
		if (r <= 0) {
			fprintf(stderr, "pos=0x%lx ", (u_long)pos);
			perror("kmemcpy:read");
			return (-1);
		} else {
			buf += r;
			pos += r;
			n -= r;
		}
	return (0);
}

int
kstrncpy(register char *buf, long pos, register int n)
{
	register int r;

	if (!n)
		return (0);

	if (kvm_f == NULL)
		if (openkmem(NULL, NULL) == -1)
			return (-1);

	while (n > 0) {
		r = kvm_read(kvm_f, pos, buf, 1);
		if (r <= 0) {
			fprintf(stderr, "pos=0x%lx ", (u_long)pos);
			perror("kmemcpy:read");
			return (-1);
		} else {
			if (*buf == '\0')
				break;
			buf++;
			pos++;
			n--;
		}
	}
	return (0);
}

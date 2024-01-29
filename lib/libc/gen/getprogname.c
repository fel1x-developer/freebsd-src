#include <stdlib.h>

#include "libc_private.h"
#include "namespace.h"
#include "un-namespace.h"

__weak_reference(_getprogname, getprogname);

const char *
_getprogname(void)
{

	return (__progname);
}

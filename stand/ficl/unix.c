
#include <netinet/in.h>

#include <string.h>

#include "ficl.h"

unsigned long
ficlNtohl(unsigned long number)
{
	return ntohl(number);
}

void
ficlCompilePlatform(FICL_DICT *dp)
{
	return;
}

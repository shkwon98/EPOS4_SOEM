#include "errorHandling.h"

void errorHandling(char *errmsg)
{
    fputs(errmsg, stderr);
    fputc('\n', stderr);
    exit(1);
}
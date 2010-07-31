#include <sys/types.h>
#include <sys/systm.h>

int hintmode = 1;
char static_hints[] = {
"hint.siba.0.at=nexus0\0"
"hint.siba.0.maddr=0x18000000\0"
"hint.siba.0.msize=0x1000\0"
"\0"
};

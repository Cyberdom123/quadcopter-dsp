#if !defined(MY_SPRINTF)
#define MY_SPRINTF

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static void ftoa_fixed(char *buffer, double value);
void sprintf_opt(char* buff, char const *fmt, ...);

#endif // MY_SPRINTF

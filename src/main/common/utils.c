#include <string.h>
#include <ctype.h>
#include "utils.h"

#ifdef _WIN32
char *strsep(char **restrict stringp, const char *restrict delim)
{
    char *s, *token, *p;
    const char *d;

    if ((s = *stringp) == NULL)
        return NULL;

    token = s;
    p = s;
    while (*p != '\0') {
        for (d = delim; *d != '\0'; ++d) {
            if (*p == *d) {
                *p = '\0';
                *stringp = p + 1;
                return token;
            }
        }
        ++p;
    }

    /* No more delimiters: return the rest of the string and set *stringp to NULL */
    *stringp = NULL;
    return token;
}

char *strcasestr(const char *haystack, const char *needle)
{
    if (!*needle)  /* empty needle matches at start */
        return (char *)haystack;

    for (; *haystack != '\0'; ++haystack) {
        const char *h = haystack;
        const char *n = needle;

        while (*h != '\0' && *n != '\0'
               && tolower((unsigned char)*h) == tolower((unsigned char)*n)) {
            ++h;
            ++n;
        }
        if (*n == '\0')  /* reached end of needle: match found */
            return (char *)haystack;
    }

    return NULL;
}
int ffs(int i)
{
    if (i == 0)
        return 0;

    unsigned int u = (unsigned int)i;
    int pos = 1;

    /* shift right until we find a 1 in the LSB */
    while ((u & 1U) == 0U) {
        u >>= 1;
        pos++;
    }

    return pos;
}
int clock_gettime(int clk_id, struct timespec *tp) {
    static int time = 0;
    ++time;
    return time;
}

int nanosleep(const struct timespec *duration, struct timespec * rem) {
  return 0;
}
#endif

#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "utils.h"

#ifdef _WIN32
#include <windows.h>

// From third_party\betaflight\lib\main\pico-sdk\lib\mbedtls\tests\suites\test_suite_x509parse.function
char *strsep(char **restrict stringp, const char *restrict delim)
{
    const char *p;
    char *ret = *stringp;

    if (*stringp == NULL) {
        return NULL;
    }

    for (;; (*stringp)++) {
        if (**stringp == '\0') {
            *stringp = NULL;
            goto done;
        }

        for (p = delim; *p != '\0'; p++) {
            if (**stringp == *p) {
                **stringp = '\0';
                (*stringp)++;
                goto done;
            }
        }
    }

done:
    return ret;
}

// From third_party\betaflight\src\main\common\string_light.c
char *strcasestr(const char *haystack, const char *needle)
{
    int nLen = strlen(needle);
    do {
        if (!strncasecmp(haystack, needle, nLen)) {
            return (char *)haystack;
        }
        haystack++;
    } while (*haystack);
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

#if !defined(USE_DETERMINISM)
typedef int clockid_t;
#define CLOCK_REALTIME                  0
#define CLOCK_MONOTONIC                 1

// From https://stackoverflow.com/a/51974214 @jws
#define MS_PER_SEC      1000ULL     // MS = milliseconds
#define US_PER_MS       1000ULL     // US = microseconds
#define HNS_PER_US      10ULL       // HNS = hundred-nanoseconds (e.g., 1 hns = 100 ns)
#define NS_PER_US       1000ULL

#define HNS_PER_SEC     (MS_PER_SEC * US_PER_MS * HNS_PER_US)
#define NS_PER_HNS      (100ULL)    // NS = nanoseconds
#define NS_PER_SEC      (MS_PER_SEC * US_PER_MS * NS_PER_US)

int clock_gettime_monotonic(struct timespec *tv)
{
    static LARGE_INTEGER ticksPerSec;
    LARGE_INTEGER ticks;

    if (!ticksPerSec.QuadPart) {
        QueryPerformanceFrequency(&ticksPerSec);
        if (!ticksPerSec.QuadPart) {
            errno = ENOTSUP;
            return -1;
        }
    }

    QueryPerformanceCounter(&ticks);

    tv->tv_sec = (long)(ticks.QuadPart / ticksPerSec.QuadPart);
    tv->tv_nsec = (long)(((ticks.QuadPart % ticksPerSec.QuadPart) * NS_PER_SEC) / ticksPerSec.QuadPart);

    return 0;
}

int clock_gettime_realtime(struct timespec *tv)
{
    FILETIME ft;
    ULARGE_INTEGER hnsTime;

    GetSystemTimePreciseAsFileTime(&ft);

    hnsTime.LowPart = ft.dwLowDateTime;
    hnsTime.HighPart = ft.dwHighDateTime;

    // To get POSIX Epoch as baseline, subtract the number of hns intervals from Jan 1, 1601 to Jan 1, 1970.
    hnsTime.QuadPart -= (11644473600ULL * HNS_PER_SEC);

    // modulus by hns intervals per second first, then convert to ns, as not to lose resolution
    tv->tv_nsec = (long) ((hnsTime.QuadPart % HNS_PER_SEC) * NS_PER_HNS);
    tv->tv_sec = (long) (hnsTime.QuadPart / HNS_PER_SEC);

    return 0;
}

int clock_gettime(clockid_t type, struct timespec *tp)
{
    if (type == CLOCK_MONOTONIC)
    {
        return clock_gettime_monotonic(tp);
    }
    else if (type == CLOCK_REALTIME)
    {
        return clock_gettime_realtime(tp);
    }

    errno = ENOTSUP;
    abort();
    return -1;
}

uint64_t to_nanoseconds(const struct timespec *timespec) {
    return timespec->tv_sec * NS_PER_SEC + timespec->tv_nsec;
}

int nanosleep(const struct timespec *duration, struct timespec * rem) {
    struct timespec start, now;
    uint64_t duration_nsec, start_nsec, now_nsec;
    duration_nsec = to_nanoseconds(duration);

    // POSIX.1 specifies that nanosleep() should measure time against the
    // CLOCK_REALTIME clock.  However, Linux measures the time using the
    // CLOCK_MONOTONIC clock.  This probably does not matter, since the
    // POSIX.1 specification for clock_settime(2) says that discontinuous
    // changes in CLOCK_REALTIME should not affect nanosleep()
    if (clock_gettime(CLOCK_MONOTONIC, &start) != 0) {
        return -1;
    }
    start_nsec = to_nanoseconds(&start);

    for (;;) {
        if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
            return -1;
        }
        now_nsec = to_nanoseconds(&now);
        uint64_t elapsed_nsec  = now_nsec - start_nsec;

        if (elapsed_nsec > duration_nsec) {
            break;
        }
    }

    // Assume because we did a spinwait that there was no remainder
    if (rem) {
        rem->tv_sec  = 0;
        rem->tv_nsec = 0;
    }
    errno = EINTR;
    return 0;
}
#endif
#endif

#if defined(USE_DETERMINISM)
#define NS_PER_SEC 1000000000ULL
#define NS_PER_US 1000ULL
#define NS_PER_FRAME (NS_PER_SEC / ((uint64_t)KF_SCHEDULER_HZ))
#define MAX_CLOCK_SAMPLES_PER_FRAME 1000ULL
#define NS_PER_CLOCK_GETTIME (NS_PER_FRAME / MAX_CLOCK_SAMPLES_PER_FRAME)

extern uint64_t externalFrame;
static uint64_t previousFrame = -1;
static uint64_t frameBaseNs = 0;
static uint64_t nsThisFrame = 0;
static uint64_t nsAdvance = 0;
static uint64_t nsTotalLast = 0;

int nanosleep_override(const struct timespec *duration, struct timespec *rem) {
    nsThisFrame += duration->tv_sec * NS_PER_SEC;
    nsThisFrame += duration->tv_nsec;

    if (rem) {
        rem->tv_sec  = 0;
        rem->tv_nsec = 0;
    }
    return 0;
}

uint64_t clock_gettime_nsec(void) {
    if (externalFrame == previousFrame) {
        nsThisFrame += NS_PER_CLOCK_GETTIME;
    } else {
        uint32_t framesAdvanced = externalFrame - previousFrame;
        previousFrame = externalFrame;

        // If we went past the typical budget for a frame, such as when delaying/waiting
        // then accumulate this advance in time by how much we went over
        uint64_t nsFramesAdvanced = NS_PER_FRAME * framesAdvanced;
        if (nsThisFrame > nsFramesAdvanced) {
            nsAdvance += nsThisFrame - nsFramesAdvanced;
        }

        nsThisFrame = 0;
        frameBaseNs = (externalFrame * NS_PER_FRAME) + nsAdvance;
    }

    uint64_t nsTotal = frameBaseNs + nsThisFrame;

    if (nsTotal < nsTotalLast)
    {
        printf("Total time went backwards %"PRIu64" %"PRIu64"\n", nsTotal, nsTotalLast);
        fflush(stdout);
        abort();
    }

    nsTotalLast = nsTotal;
    return nsTotal;
}

int clock_gettime_override(clockid_t clk_id, struct timespec *tp) {
    uint64_t nsTotal = clock_gettime_nsec();

    UNUSED(clk_id);

    tp->tv_sec = nsTotal / NS_PER_SEC;
    tp->tv_nsec = nsTotal % NS_PER_SEC;
    return 0;
}
#endif

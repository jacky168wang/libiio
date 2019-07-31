/*
 * applications based on libiio
 *   - type definitions and prototype declarations
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Jacky Wang <kenwj@sina.com>
 *
 * License: GPL, version 2.1
 */
#ifndef _COMMON_JACKY_H
#define _COMMON_JACKY_H

/*==================================
*/
#ifndef ASSERT
#define ASSERT(expr) { \
		if (!(expr)) { \
			fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
			abort(); \
		} \
	}
#endif

/*==================================
*/
#ifndef max
#define max(a,b)	(((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)	(((a) < (b)) ? (a) : (b))
#endif

static inline long myabs(long x)
{
	return x < 0 ? -x : x;
}

static inline int is_bigendian()
{
    int i=1;
    return ! *((char *)&i);
}

/*==================================
*/
#include <time.h>
static inline double elapse_ms(struct timespec *e, struct timespec *s)
{
	return (e->tv_sec - s->tv_sec)*1e3 + (e->tv_nsec - s->tv_nsec)/1e6;
}

static inline double elapse_us(struct timespec *e, struct timespec *s)
{
    return (e->tv_sec - s->tv_sec)*1e6 + (e->tv_nsec - s->tv_nsec)/1e3;
}

static inline double elapse_ns(struct timespec *e, struct timespec *s)
{
	return (e->tv_sec - s->tv_sec)*1e9 + (e->tv_nsec - s->tv_nsec);
}

// cannot be called 2+times by the same thread
static inline void measure_interval(unsigned long ius, char *owner)
{
    static unsigned long handler_cnt=0;
	static struct timespec old;
    struct timespec new;
	double tm_us;

    if (handler_cnt == 0) {
        clock_gettime(CLOCK_MONOTONIC, &old);
    } else {
        clock_gettime(CLOCK_MONOTONIC, &new);
        tm_us = elapse_us(&new, &old);
        if (tm_us > ius*1.5) {
            printf("%s[%lu]: interval accuracy +50%+\n",
                owner, handler_cnt);
        } else if (tm_us > ius*1.4) {
            printf("%s[%lu]: interval accuracy +40~50%\n",
                owner, handler_cnt);
        } else if (tm_us > ius*1.3) {
            printf("%s[%lu]: interval accuracy +30~40%\n",
                owner, handler_cnt);
#if 0
        } else if (tm_us > ius*1.2) {
            printf("%s[%lu]: interval accuracy +20~30%\n",
                owner, handler_cnt);
        } else if (tm_us > ius*1.1) {
            printf("%s[%lu]: interval accuracy +10~20%\n",
                owner, handler_cnt);
        } else if (tm_us > ius*1.05) {
            printf("%s[%lu]: interval accuracy +5~10%\n",
                owner, handler_cnt);
        } else if (tm_us < ius*1.025) {
            printf("%s[%lu]: interval accuracy +2.5%~5%\n",
                owner, handler_cnt);
#endif
        } else if (tm_us < ius*0.5) {
            printf("%s[%lu]: interval accuracy -50%+\n",
                owner, handler_cnt);
        } else if (tm_us < ius*0.6) {
            printf("%s[%lu]: interval accuracy -40~50%\n",
                owner, handler_cnt);
        } else if (tm_us < ius*0.7) {
            printf("%s[%lu]: interval accuracy -30~40%\n",
                owner, handler_cnt);
#if 0
        } else if (tm_us < ius*0.8) {
            printf("%s[%lu]: interval accuracy -20~10%\n",
                owner, handler_cnt);
        } else if (tm_us < ius*0.9) {
            printf("%s[%lu]: interval accuracy -10~20%\n",
                owner, handler_cnt);
        } else if (tm_us < ius*0.95) {
            printf("%s[%lu]: interval accuracy -5~10%\n",
                owner, handler_cnt);
        } else if (tm_us < ius*0.975) {
            printf("%s[%lu]: interval accuracy -2.5~5%\n",
                owner, handler_cnt);
#endif
        }
#if 1
        old = new;
#else
        lock();
        old.tv_sec  = new.tv_sec ;
        old.tv_nsec = new.tv_nsec;
        unlock();
#endif
    }
    handler_cnt++;
}

/*==================================
*/

static inline void macaddr_s2a(char a[], const char *s)
{
	unsigned int i, value[6];
	sscanf(s, "%x:%x:%x:%x:%x:%x", &value[0], &value[1], \
		&value[2], &value[3], &value[4], &value[5]);
	for (i=0; i<6; i++) a[i] = (char) value[i];
}
static inline void macaddr_a2s(char *s, const char a[])
{
	sprintf(s, "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], 
		a[2], a[3], a[4], a[5]);
}
static inline void macaddr_print(const char a[])
{
	printf("%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], 
		a[2], a[3], a[4], a[5]);
}

#endif /* _COMMON_JACKY_H */

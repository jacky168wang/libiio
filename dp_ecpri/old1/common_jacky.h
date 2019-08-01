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

static inline unsigned int myabs(int x)
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
	return 1e3*(e->tv_sec - s->tv_sec) + (e->tv_nsec - s->tv_nsec)/1e6;
}
static inline double elapse_us(struct timespec *e, struct timespec *s)
{
	return 1e6*(e->tv_sec - s->tv_sec) + (e->tv_nsec - s->tv_nsec)/1e3;
}
static inline double elapse_ns(struct timespec *e, struct timespec *s)
{
	return 1e9*(e->tv_sec - s->tv_sec) + e->tv_nsec - s->tv_nsec;
}

static inline void macaddr_s2a(uint8_t a[], const char *s)
{
	unsigned int i, value[6];
	sscanf(s, "%x:%x:%x:%x:%x:%x", &value[0], &value[1], \
		&value[2], &value[3], &value[4], &value[5]);
	for (i=0; i<6; i++) a[i] = (uint8_t) value[i];
}
static inline void macaddr_a2s(char *s, const uint8_t a[])
{
	sprintf(s, "%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], 
		a[2], a[3], a[4], a[5]);
}
static inline void macaddr_print(const uint8_t a[])
{
	printf("%02x:%02x:%02x:%02x:%02x:%02x", a[0], a[1], 
		a[2], a[3], a[4], a[5]);
}

#endif /* _COMMON_JACKY_H */

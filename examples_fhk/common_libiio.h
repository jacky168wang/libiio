/*
 * applications based on libiio
 *   - type definitions and prototype declarations
 *
 * Copyright (C) 2018~2020 FHK Inc.
 * Author: Jacky Wang <kenwj@sina.com>
 *
 * License: GPL, version 2.1
 */
#ifndef _COMMON_LIBIIO_H
#define _COMMON_LIBIIO_H

/* static scratch mem for strings */
#define MAXSLEN  64

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

/* NOTICE: check the iio driver: RX is input, TX is output */
enum iopath { RX=0, TX=1, OR=2 };
static const char *str_path[] = {
	"RX", "TX", "OR"
};

/* RFIC(iio-device: phy) */
#define TOT_CHIP_NB 1	/* total chip number: TODO: 4 */
#define DEV_PORT_NB 1	/* each chip has 2-RX & 2-TX ports: TODO: 2 */
#define TOT_PORT_NB (TOT_CHIP_NB*DEV_PORT_NB)
#define CHN_PER_PRT 2	/* each RX/TX port has one pair of I/Q channels */

/*
iio-internal-buffer is 4MiB at most (per libiio spec.) so 
	  1 MiSample when 1-port(i.e. 2-channel)
	512 KiSample when 2-port(i.e. 4-channel)
*/
#define IIO_SMP_MAX ((1 << 20) / DEV_PORT_NB)

/* print libiio version */
static inline void dmp_libiio_version(void)
{
    unsigned int major, minor;
	char git_tag[8];

	iio_library_get_version(&major, &minor, git_tag);
	fprintf(stdout, "libiio version: %u.%u (git tag: %s)\r\n\r\n",
		major, minor, git_tag);
}

/* check return value of attr_write function */
static inline void ERRCHK(int v, const char* what)
{
	if (v < 0) {
		fprintf(stderr, "ERR: access iio-attr '%s' failed with %d\r\n",
			what, v);
		exit(0);
	}
}

#if 1
/* write attribute: long long int */
static inline void W_CHN_LLI(struct iio_channel *chn, const char* what, long long val)
{
	ERRCHK(iio_channel_attr_write_longlong(chn, what, val), what);
}
/* read  attribute: long long int */
static inline long long R_CHN_LLI(struct iio_channel *chn, const char* what)
{
	long long val;
	ERRCHK(iio_channel_attr_read_longlong(chn, what, &val), what);
	if (g_m_verbo > 1)
		fprintf(stdout, "INF: iio-chn[%s] iio-attr '%s'=%lld\r\n",
			chn->name, what, val);
	return val;
}

/* write attribute: double */
static inline void W_CHN_DBL(struct iio_channel *chn, const char* what, double val)
{
	ERRCHK(iio_channel_attr_write_double(chn, what, val), what);
}

/* read  attribute: double */
static inline double R_CHN_DBL(struct iio_channel *chn, const char* what)
{
	double val;
	ERRCHK(iio_channel_attr_read_double(chn, what, &val), what);
	if (g_m_verbo > 1)
		fprintf(stdout, "INF: iio-chn[%s] iio-attr '%s'=%f\r\n",
			chn->name, what, val);
	return val;
}

/* write attribute: bool */
static inline void W_CHN_BOO(struct iio_channel *chn, const char* what, bool val)
{
	ERRCHK(iio_channel_attr_write_bool(chn, what, val), what);
}

/* read  attribute: bool */
static inline double R_CHN_BOO(struct iio_channel *chn, const char* what)
{
	bool val;
	ERRCHK(iio_channel_attr_read_bool(chn, what, &val), what);
	if (g_m_verbo > 1)
		fprintf(stdout, "INF: iio-chn[%s] iio-attr '%s'=%d\r\n",
			chn->name, what, val);
	return val;
}

/* write attribute: string */
static inline void W_CHN_STR(struct iio_channel *chn, const char* what, const char* str)
{
	ERRCHK(iio_channel_attr_write(chn, what, str), what);
}
/* read  attribute: string */
static inline const char* R_CHN_STR(struct iio_channel *chn, const char* what)
{
	const char* p;
	ERRCHK(iio_channel_attr_read(chn, what, str), what);
}

#else
/* helper function to check return value of attr_write function */
/* int chn, const char* type_string, const char* what, type_key value */
#define IIO_ATTR(A, C, T, W, V) \
do {\
	int ret;\
	ret = iio_channel_attr_##A_##T(C, W, V);\
	if (ret < 0) {\
		fprintf(stderr, "ERR: "#A" iio-attr "#W" failed with %d\r\n", ret);\
		abort();\
	}\
} while (0);
#endif

/* write attribute: long long int */
static inline void W_DBG_LLI(struct iio_device *dev, const char* what, long long val)
{
	ERRCHK(iio_device_debug_attr_write_longlong(dev, what, val), what);
}
/* read  attribute: long long int */
static inline long long R_DBG_LLI(struct iio_device *dev, const char* what)
{
	long long val;
	ERRCHK(iio_device_debug_attr_read_longlong(dev, what, &val), what);
	if (g_m_verbo > 1)
		fprintf(stdout, "INF: iio-dev[%s] iio-attr '%s'=%lld\r\n",
			dev->name, what, val);
	return val;
}

/* helper function generating channel names */
static char tmpstr[MAXSLEN];
static inline char* GET_CHN_NAME(const char* type, int id, char modify)
{
	if ('\0' == modify)
		snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	else
		snprintf(tmpstr, sizeof(tmpstr), "%s%d_%c", type, id, modify);
	return tmpstr;
}

#endif

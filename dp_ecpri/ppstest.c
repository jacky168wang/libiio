/*
 * ppstest.c -- simple tool to monitor PPS timestamps
 *
 * Copyright (C) 2005-2007   Rodolfo Giometti <giometti@linux.it>
 *
 * License: GPL, version 2.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "timepps.h"
#include "common_jacky.h"
#if defined(BUILDING_RRU_UL)
#include "rru_bbu.h"
# if !defined(TMPPS_TRG_KSIGIO_NOWAIT)
#  error "TMPPS_TRG_KSIGIO_NOWAIT should be defined!"
# endif
extern void tmpps_handler(int signo);
#endif

int pps_find_src(char *path, pps_handle_t *handle, int *avail_mode)
{
	pps_params_t params;
	int ret;

	printf("trying PPS source \"%s\"\n", path);

	/* Try to find the source by using the supplied "path" name */
	ret = open(path, O_RDWR);
	if (ret < 0) {
		fprintf(stderr, "unable to open device \"%s\" (%m)\n", path);
		return ret;
	}

	/* Open the PPS source (and check the file descriptor) */
	ret = time_pps_create(ret, handle);
	if (ret < 0) {
		fprintf(stderr, "cannot create a PPS source from device "
				"\"%s\" (%m)\n", path);
		return -1;
	}
	printf("found PPS source \"%s\"\n", path);

	/* Find out what features are supported */
	ret = time_pps_getcap(*handle, avail_mode);
	if (ret < 0) {
		fprintf(stderr, "cannot get capabilities (%m)\n");
		return -1;
	}
    printf("capabilities 0x%x\n", *avail_mode);
	if ((*avail_mode & PPS_CAPTUREASSERT) == 0) {
		fprintf(stderr, "cannot CAPTUREASSERT\n");
		return -1;
	}
	if ((*avail_mode & PPS_OFFSETASSERT) == 0) {
		fprintf(stderr, "cannot OFFSETASSERT\n");
		return -1;
	}

	/* Capture assert timestamps, and compensate for a 675 nsec
	 * propagation delay */
	ret = time_pps_getparams(*handle, &params);
	if (ret < 0) {
		fprintf(stderr, "cannot get parameters (%m)\n");
		return -1;
	}
#if !defined(BUILDING_RRU_UL)
	params.assert_offset.tv_sec = 0;
	params.assert_offset.tv_nsec = 675;
    params.mode |= PPS_CAPTUREASSERT | PPS_OFFSETASSERT;
#else
    params.mode |= PPS_CAPTUREASSERT;
#endif
	ret = time_pps_setparams(*handle, &params);
	if (ret < 0) {
		fprintf(stderr, "cannot set parameters (%m)\n");
		return -1;
	}

	return 0;
}

int pps_fetch_src(int i, pps_handle_t *handle, int *avail_mode)
{
	struct timespec timeout;
	pps_info_t info;
	int ret;
    static unsigned long handler_cnt=0;
	struct timespec old, new;
    unsigned long abs;

	/* create a zero-valued timeout */
	timeout.tv_sec = 3;
	timeout.tv_nsec = 0;

retry:
	if (*avail_mode & PPS_CANWAIT) { /* waits/blocks for the next event */
		ret = time_pps_fetch(*handle, PPS_TSFMT_TSPEC, &info,
				   &timeout);
	} else {
        printf("sleep as 'PPS_CANWAIT' not supported\n"); fflush(stdout);
#if !defined(BUILDING_RRU_UL)
		sleep(1);
#else
        usleep(1000);
#endif
		ret = time_pps_fetch(*handle, PPS_TSFMT_TSPEC, &info,
				   &timeout);
	}
	if (ret < 0) {
		if (ret == -EINTR) {
			fprintf(stderr, "time_pps_fetch() got a signal!\n");
			goto retry;
		}

		fprintf(stderr, "time_pps_fetch() error %d (%m)\n", ret);
		return -1;
	}

#if 0
    printf("pps%d: assert[%ld] %ld.%09ld\n",
           i,
           info.assert_sequence,
           info.assert_timestamp.tv_sec,
           info.assert_timestamp.tv_nsec);
    fflush(stdout);
#else
# if !defined(TMPPS_TRG_KSIGIO_NOWAIT)
    measure_interval(1000, "pps_fetch_src");
# endif
#endif

	return 0;
}


#if !defined(BUILDING_RRU_UL)
void usage(char *name)
{
	fprintf(stderr, "usage: %s <ppsdev> [<ppsdev> ...]\n", name);
	exit(EXIT_FAILURE);
}

int main(int argc, char *argv[])
{
	int num;
	pps_handle_t handle[4];
	int avail_mode[4];
	int i = 0;
	int ret;

	/* Check the command line */
	if (argc < 2)
		usage(argv[0]);

	for (i = 1; i < argc && i <= 4; i++) {
		ret = pps_find_src(argv[i], &handle[i - 1], &avail_mode[i - 1]);
		if (ret < 0)
			exit(EXIT_FAILURE);
	}

	num = i - 1;
	printf("ok, found %d source(s), now start fetching data...\n", num);

	/* loop, printing the most recent timestamp every second or so */
	while (1) {
		for (i = 0; i < num; i++) {
			ret = pps_fetch_src(i, &handle[i], &avail_mode[i]);
			if (ret < 0 && errno != ETIMEDOUT)
				exit(EXIT_FAILURE);
		}
	}

	for (; i >= 0; i--)
		time_pps_destroy(handle[i]);

	return 0;
}
#else
pps_handle_t pps_handle;
int pps_mode;

void *task_tmpps_pps(void *parg)
{
    int ret;
    //unused(parg);

    ret = pps_find_src("/dev/pps0", &pps_handle, &pps_mode);
    if (ret < 0)
        exit(EXIT_FAILURE);

    /* loop, printing the most recent timestamp every second or so */
    do {
        ret = pps_fetch_src(0, &pps_handle, &pps_mode);
        if (ret < 0 && errno != ETIMEDOUT)
            exit(EXIT_FAILURE);
        tmpps_handler(0);
    } while (1);

    time_pps_destroy(pps_handle);

    return 0;
}
#endif

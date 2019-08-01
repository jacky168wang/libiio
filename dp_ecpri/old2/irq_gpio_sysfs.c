/*
#1> change your devicetree to add one gpio-irq like the following
================================================================================
Reference:
    http://www.wowotech.net/device_model/why-dt.html
    https://elinux.org/Device_Tree_Usage
    http://xillybus.com/tutorials/device-tree-zynq-1

specific-devicetree:
------------------------------------
	//Documentation/devicetree/bindings/gpio/gpio-altera.txt
	//Documentation/devicetree/bindings/interrupt-controller/arm,gic.txt
	gpio_altr: gpio@0xff200000 {
	   compatible = "altr,pio-1.0";
	   reg = <0xff200000 0x10>;
	   //interrupts = <0 45 4>;//<irq-type irq-number trigger/level-flag>
	   interrupts = <0 24 1>;//<irq-type irq-number trigger/level-flag>
	   altr,ngpio = <32>;
	   altr,interrupt-type = <IRQ_TYPE_EDGE_RISING>;
	   #gpio-cells = <2>;
	   gpio-controller;
	   #interrupt-cells = <1>;
	   interrupt-controller;
	};

	//arch/arm/boot/dts/zynq-7000.dtsi
	gpio0: gpio@e000a000 {
		compatible = "xlnx,zynq-gpio-1.0";
		#gpio-cells = <2>;
		clocks = <&clkc 42>;
		gpio-controller;
		interrupt-controller;
		#interrupt-cells = <2>;
		interrupt-parent = <&intc>;
		interrupts = <0 20 4>;
		reg = <0xe000a000 0x1000>;
	};

	//arch/arm/boot/dts/zynq-zc706-adv7511-fmcdaq2.dts
	&adc0_ad9680 {
	    powerdown-gpios = <&gpio0 96 0>;
	    fastdetect-a-gpios = <&gpio0 89 0>;
	    fastdetect-b-gpios = <&gpio0 90 0>;
	};

#2> understand the interrupt principle and linux interrupt subsystem/framework
================================================================================
Reference:
    http://www.wowotech.net/sort/irq_subsystem

#3> understand the linux gpio driver and gpiolib framework
================================================================================
Reference:
    Documentation/gpio.txt
	https://elinux.org/EBC_Exercise_11_gpio_Polling_and_Interrupts
	https://developer.ridgerun.com/wiki/index.php/How_to_use_GPIO_signals
	https://developer.toradex.com/knowledge-base/gpio-(linux)

Linux Driver for GPIO controller: 'gpio_chip'
------------------------------------
  - methods to establish GPIO direction
  - methods used to access GPIO values
  - flag saying whether calls to its methods may sleep
  - optional debugfs dump method (showing extra state like pullup config)
  - label for diagnostics

Linux LEDs Driver depending on GPIO: 'leds-gpio'
------------------------------------

Linux Buttons Driver depending on GPIO: 'gpio_keys'
------------------------------------

BSP(Board Support Package) efforts:
------------------------------------
  1, create structures identifying the range of GPIOs that the specific GPIO
     controllers will expose
  2, passes them to each GPIO expander chip using platform_data
  3, the chip driver's probe() pass that data to gpiochip_add()

Linux GPIO Framework/Subsystem: 'gpiolib'
------------------------------------
  Goal: support kinds of GPIO controller using the same programming interface
  debugfs: /sys/kernel/debug/gpio :
    list all the controllers registered through this framework
  Kconfig options:
    ARCH_REQUIRE_GPIOLIB: always get compiled into the kernel
    ARCH_WANT_OPTIONAL_GPIOLIB: defaults to off and can be built optionally
  platform support:
    integrate platform-specific code to register 'gpio_chip' instances into.
    platform initialization from arch_initcall().

#4> Userspace
================================================================================
#4.1> Sysfs Interface for Userspace:
------------------------------------
  0, add the GPIO controller into your dts; add the corresponding driver in the kernel
  1, See which GPIO pins are reserved by different kernel drivers
  console> cat /sys/kernel/debug/gpio
  2, See what GPIO controllers are managed by 'pinctrl' subsystem/framework
  # ls /sys/class/gpio/
  console> export  gpiochip451@  unexport
  3, Export the specified GPIO-IRQ (confirm your HW design) to "sysfs"
  console> echo 451+15 > /sys/class/gpio/export (gpio466 will be created)
  4, Set the signal direction
  #console> echo in > /sys/class/gpio/gpio466/direction
  5, See 'edge' exists
  # only after the pin has been configured as an interrupt generating input pin
  console> ls /sys/class/gpio/gpio511/edge
  6, Set the signal edge if edge-trigger
  console> echo <rising,falling,both> > /sys/class/gpio/gpio511/edge
  7, Set the signal level if level-trigger
  console> echo <rising,falling,both> > /sys/class/gpio/gpio511/edge
  8, userspace detect IRQ by using the poll(2) on /sys/class/gpio/gpio466/value
  8.1, set the events POLLPRI and POLLERR
  8.2, poll(2) will block until the specified edge is asserted
  8.3, lseek(2)-then-read-'value' or close-open-'value' to read the new value

#5> man poll/ppoll
------------------------------------
  like pselect(2), ppoll() allows an application to safely wait until either a file descriptor becomes ready
  or until a signal is caught.
    {
      ready = ppoll(&fds, nfds, timeout_ts, &sigmask);
    }
  is equivalent to atomically executing the following calls:
    {
      sigset_t origmask;
      int timeout;
      timeout = (timeout_ts == NULL) ? -1 :
      (timeout_ts.tv_sec * 1000 + timeout_ts.tv_nsec / 1000000);
      sigprocmask(SIG_SETMASK, &sigmask, &origmask);
      ready = poll(&fds, nfds, timeout);
      sigprocmask(SIG_SETMASK, &origmask, NULL);
    }
*/

/* example1:
------------------------------------
  Reference:
    http://www.wiki.xilinx.com/Linux+GPIO+Driver
    http://www.wiki.xilinx.com/GPIO+User+Space+App
  Kconfig options:
    CONFIG_GPIO_SYSFS=y
    CONFIG_SYSFS=y
    CONFIG_EXPERIMENTAL=y
    CONFIG_GPIO_XILINX=y
  Performance on the ML507 reference system:
    Pretty good: the GPIO can be toggled about every 4 usec
  Commands to setup the GPIO:
    console> mount -t sysfs sysfs /sys
    console> echo 240 > /sys/class/gpio/export
    console> echo out > /sys/class/gpio/gpio240/direction
    console> echo 1 > /sys/class/gpio/gpio240/value
  Bash-script to toggle the gpio for testing
    while [ 1 ]; do
      echo 1 > /sys/class/gpio/gpio240/value
      echo 0 > /sys/class/gpio/gpio240/value
    done
*/

/* gpio-int-test.c
 *
 * Copyright (c) 2011, RidgeRun
 * All rights reserved.
 *
 * Contributors include:
 *	 Todd Fischer
 *	 Brad Lu
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *	  must display the following acknowledgement:
 *	  This product includes software developed by the RidgeRun.
 * 4. Neither the name of the RidgeRun nor the
 *	  names of its contributors may be used to endorse or promote products
 *	  derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY RIDGERUN ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL RIDGERUN BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#if defined(BUILDING_RRU_UL)
#include "rru_bbu.h"
# if !defined(TMPPS_TRG_UGPIOIRQ)
#  error "TMPPS_TRG_UGPIOIRQ should be defined!"
# endif
#endif

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

int gpio_export(int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);

	return 0;
}

int gpio_unexport(int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return fd;
	}

	len = snprintf(buf, sizeof(buf), "%d", gpio);
	write(fd, buf, len);
	close(fd);
	return 0;
}

int gpio_set_dir(int gpio, int out_flag)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-dir");
		return fd;
	}

	write(fd, out_flag ? "out" : "in", 4);
	close(fd);
	return 0;
}

int gpio_set_value(int gpio, int true_flag)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return fd;
	}

	write(fd, true_flag ? "1" : "0", 2);
	close(fd);
	return 0;
}

int gpio_get_value(int gpio, int *value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}

	read(fd, &ch, 1);
	*value = (ch != '0') ? 1 : 0;
	close(fd);
	return 0;
}

int gpio_set_edge(int gpio, char *edge)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}

	write(fd, edge, strlen(edge) + 1); 
	close(fd);
	return 0;
}

int gpio_set_level(int gpio, int high_flag)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/active_low", gpio);
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-level");
		return fd;
	}

	write(fd, high_flag ? "0" : "1", 2);
	close(fd);
	return 0;
}

int gpio_fd_open(int gpio, int level_or_edge)
{
	int fd, len;
	char buf[MAX_BUF];

	gpio_export(gpio);
	gpio_set_dir(gpio, 0);
	if (level_or_edge)
		gpio_set_level(gpio, 1);
	else
		gpio_set_edge(gpio, "rising");

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
	fd = open(buf, O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		perror("gpio/open-value");
	}
	return fd;
}

int gpio_fd_close(int fd)
{
	return close(fd);
}

#if !defined(BUILDING_RRU_UL)
int main(int argc, char **argv, char **envp)
{
	int ec, fdgpio, mode;
	struct pollfd fdset[2];
	int nfds = sizeof(fdset)/sizeof(fdset[0]);
	char buf[MAX_BUF];
    char str[16];
    
	if (argc < 3) {
		printf("Usage: gpio-int <gpio-pin> <level/edge-trigger> <mode>\n\n"
				"\tWaits for a change in the GPIO pin or input on stdin\n");
		exit(-1);
	}
	mode = (argc < 3) ? 0 : atoi(argv[3]);
    sprintf(str, "IRQGPIO%d", atoi(argv[1]));

	fdgpio = gpio_fd_open(atoi(argv[1]), atoi(argv[2]));
	if (fdgpio < 0) exit(-1);

	while (1) {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = fdgpio;
		fdset[0].events = POLLPRI;
		fdset[1].fd = STDIN_FILENO;
		fdset[1].events = POLLIN;

		ec = poll(fdset, nfds, POLL_TIMEOUT);
		if (ec < 0) {
			perror("poll");
            gpio_fd_close(fdgpio);
            exit(-1);
		} else if (ec == 0) {/*timeout*/
			printf(".");
            fflush(stdout);
			continue;
		}

		if (fdset[0].revents & POLLPRI) {
			/* the following is a must to update kernel-events */
			lseek(fdset[0].fd, 0, SEEK_SET);
			ec = read(fdset[0].fd, buf, sizeof(buf));
			switch (mode) {
			default:
				printf("%s: value=%c\n", str, buf[0]);
				break;
			case 1:
				clock_gettime(CLOCK_MONOTONIC, &new);
				printf("%s: time=%lu.%lu\n", str, new.tv_nsec, new.tv_nsec);
				break;
			case 2:
                measure_interval(TMPPS_INTERVAL_US, str);
				break;
			}
		}

		if (fdset[1].revents & POLLIN) {
			lseek(fdset[1].fd, 0, SEEK_SET);
			(void) read(fdset[1].fd, buf, 1);
			printf("stdin POLLIN: %s\n", buf);
            fflush(stdout);
		}
	}

	gpio_fd_close(fdgpio);
    exit(0);
}
#else
static unsigned int mode_verbose = 0;
#define PPS_IRQ_GPIO_NUM 511
#define PPS_IRQ_TRIGER_LEVEL 0
void *task_tmpps_irq(void *parg)
{
	struct pollfd fdset[1];
	int nfds = sizeof(fdset)/sizeof(fdset[0]);
	char buf[MAX_BUF];
	int fdgpio, ec;
    //unused(parg);

    printf("task_tmpps_irq start: init GPIO%d_IRQ %s\n",
        PPS_IRQ_GPIO_NUM,
        PPS_IRQ_TRIGER_LEVEL ? "level-high" : "edge-rising");
	fdgpio = gpio_fd_open(PPS_IRQ_GPIO_NUM, PPS_IRQ_TRIGER_LEVEL);
	if (fdgpio < 0) return (void *)-1;

	do {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = fdgpio;
		fdset[0].events = POLLPRI;
		ec = poll(fdset, nfds, 3);/* 3ms */
		if (ec < 0) {
			perror("task_tmpps_irq: poll");
			return (void *)-1;
		} else if (ec == 0) {
			if (mode_verbose > 2) {
				printf("task_tmpps_irq: GPIO%d_IRQ wait %s timeout!\n",
					PPS_IRQ_GPIO_NUM,
					PPS_IRQ_TRIGER_LEVEL ? "level-high" : "edge-rising");
			}
			continue;
		}

		if (fdset[0].revents & POLLPRI) {
			/* the following is a must to update kernel-events */
			lseek(fdset[0].fd, 0, SEEK_SET);
			ec = read(fdset[0].fd, buf, sizeof(buf));

			tmpps_handler(0);
		}
	} while (!taskstop_tmirq);

	gpio_fd_close(fdgpio);
	printf("task_tmpps_irq   end\n");
	return (void *)0;
}
#endif

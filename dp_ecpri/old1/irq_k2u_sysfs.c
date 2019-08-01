/* 
Reference for devicetree
	http://www.wowotech.net/device_model/why-dt.html
	https://elinux.org/Device_Tree_Usage
	http://xillybus.com/tutorials/device-tree-zynq-1

Reference for interrupt-subsystem
	http://www.wowotech.net/sort/irq_subsystem
*/
/*
#1> change your devicetree to add one gpio-irq like the following
	//Documentation/devicetree/bindings/gpio/gpio-altera.txt
	//Documentation/devicetree/bindings/interrupt-controller/arm,gic.txt
	gpio_altr: gpio@0xff200000 {
	   compatible = "altr,pio-1.0";
	   reg = <0xff200000 0x10>;
	   interrupts = <0 45 4>;//<irq-type irq-number trigger/level-flag>
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

#2> understand the gpio driver and gpiolib framework
	//Documentation/gpio.txt
Linux Driver for GPIO controller: 'gpio_chip'
------------------------------------
	- methods to establish GPIO direction
	- methods used to access GPIO values
	- flag saying whether calls to its methods may sleep
	- optional debugfs dump method (showing extra state like pullup config)
	- label for diagnostics

Linux LEDs Driver depending on GPIO: 'leds-gpio'
Linux Buttons Driver depending on GPIO: 'gpio_keys'

BSP(Board Support Package) efforts:
------------------------------------
	1, create structures identifying the range of GPIOs that the specific GPIO controllers will expose
	2, passes them to each GPIO expander chip using platform_data
	3, the chip driver's probe() pass that data to gpiochip_add()

Linux GPIO Framework/Subsystem: 'gpiolib'
------------------------------------
	Goal: support different kinds of GPIO controller using the same programming interface
	debugfs: /sys/kernel/debug/gpio : list all the controllers registered through this framework
	Kconfig options:
		ARCH_REQUIRE_GPIOLIB: gpiolib will always get compiled into the kernel on that architecture.
		ARCH_WANT_OPTIONAL_GPIOLIB: gpiolib defaults to off and can be built into the kernel optionally.
	platform support:
		integrate platform-specific code for defining and registering 'gpio_chip' instances into
		platform initialization from arch_initcall().

#3> Sysfs Interface for Userspace:
------------------------------------
	0, add GPIO controller into your dts; add the corresponding driver in the kernel
	1.1, See which GPIO pins are reserved by different kernel drivers
	# cat /sys/kernel/debug/gpio
	1.2, See what GPIO controllers are managed by kernel 'pinctrl' subsystem/framework
	# ls /sys/class/gpio/
	export  gpiochip451@  unexport
	2, Export the specified GPIO-IRQ (confirm your HW design) to "sysfs"
	# echo 451+15 > /sys/class/gpio/export (/sys/class/gpio/gpio466 will be created)
	3, Set the signal direction
	# echo in > /sys/class/gpio/gpio466/direction
	4, (exists only if the pin can be configured as an interrupt generating input pin)
	4.1, Set the signal edge if edge-trigger
	# echo <rising,falling,both> > /sys/class/gpio/gpio511/edge
	4.2, Set the signal level if level-trigger
	# echo <rising,falling,both> > /sys/class/gpio/gpio511/edge
	5, userspace detect IRQ by using the poll(2) system-call on /sys/class/gpio/gpio466/value
	5.1, set the events POLLPRI and POLLERR
	5.2, poll(2) will block until the specified edge is asserted.
	5.3, after poll(2) returns, either lseek(2) then read /sys/class/gpio/gpio466/value, 
	or close and re-open it to read the new value.

#4> man poll/ppoll
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

#if 0
Reference:
	http://www.wiki.xilinx.com/Linux+GPIO+Driver
	http://www.wiki.xilinx.com/GPIO+User+Space+App

// The specific GPIO being used must be setup and replaced thru
// this code.  The GPIO of 240 is in the path of most the sys dirs
// and in the export write.
//
// Figuring out the exact GPIO was not totally obvious when there
// were multiple GPIOs in the system. One way to do is to go into
// the gpiochips in /sys/class/gpio and view the label as it should
// reflect the address of the GPIO in the system. The name of the
// the chip appears to be the 1st GPIO of the controller.
//
// The export causes the gpio240 dir to appear in /sys/class/gpio.
// Then the direction and value can be changed by writing to them.
 
// The performance of this is pretty good, using a nfs mount,
// running on open source linux, on the ML507 reference system,
// the GPIO can be toggled about every 4 usec.
 
// The following commands from the console setup the GPIO to be
// exported, set the direction of it to an output and write a 1
// to the GPIO.
//
// bash> echo 240 > /sys/class/gpio/export
// bash> echo out > /sys/class/gpio/gpio240/direction
// bash> echo 1 > /sys/class/gpio/gpio240/value
 
// if sysfs is not mounted on your system, the you need to mount it
// bash> mount -t sysfs sysfs /sys
 
// the following bash script to toggle the gpio is also handy for
// testing
//
// while [ 1 ]; do
//  echo 1 > /sys/class/gpio/gpio240/value
//  echo 0 > /sys/class/gpio/gpio240/value
// done
 
// to compile this, use the following command
// gcc gpio.c -o gpio
 
// The kernel needs the following configuration to make this work.
//
// CONFIG_GPIO_SYSFS=y
// CONFIG_SYSFS=y
// CONFIG_EXPERIMENTAL=y
// CONFIG_GPIO_XILINX=y
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> 

int main()
{
    int valuefd, exportfd, directionfd;
 
    printf("GPIO test running...\r\n");
 
    // The GPIO has to be exported to be able to see it
    // in sysfs
    exportfd = open("/sys/class/gpio/export", O_WRONLY);
    if (exportfd < 0) {
        printf("Cannot open GPIO to export it\r\n");
        exit(1);
    }
    write(exportfd, "240", 4);
    close(exportfd);
    printf("GPIO exported successfully\r\n");
 
    // Update the direction of the GPIO to be an output
    directionfd = open("/sys/class/gpio/gpio240/direction", O_RDWR);
    if (directionfd < 0) {
        printf("Cannot open GPIO direction it\r\n");
        exit(1);
    }
    write(directionfd, "out", 4);
    close(directionfd);
    printf("GPIO direction set as output successfully\r\n");
 
    // Get the GPIO value ready to be toggled
    valuefd = open("/sys/class/gpio/gpio240/value", O_RDWR);
    if (valuefd < 0) {
        printf("Cannot open GPIO value\r\n");
        exit(1);
    }

    printf("GPIO value opened, now toggling...\r\n");
 
    // toggle the GPIO as fast a possible forever, a control c is needed
    // to stop it
    while (1) {
        write(valuefd, "1", 2);
        write(valuefd, "0", 2);
    }
}

#endif

#if 0
	https://elinux.org/EBC_Exercise_11_gpio_Polling_and_Interrupts
	https://developer.ridgerun.com/wiki/index.php/How_to_use_GPIO_signals
		
#endif

#if 0
Reference:
	https://developer.toradex.com/knowledge-base/gpio-(linux)
#endif

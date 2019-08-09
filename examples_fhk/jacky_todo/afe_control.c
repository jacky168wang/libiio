#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

/* declaration of corresponding pointer*/
static int memfd = -1;
static void *phyaddr_this_base = NULL;
static void *phyaddr_io_orx_cfg = NULL;
static void *phyaddr_io_mod_sel = NULL;
static void *phyaddr_ip_dma_cfg = NULL;
static void *phyaddr_ip_syn_cfg = NULL;

static void usage(int argc, char *argv[]) 
{
	printf("Usage: %s <phys_addr> <offset>\n", argv[0]);
	return 0;
}

void *uspace_open_mem(off_t dev_base, size_t size)
{
    void *base;
    size_t mask = size - 1;

    if (memfd == -1) {
        memfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (memfd == -1) {
            printf("Can't open /dev/mem\n");
            return NULL;
        }
        if (mode_verbo < 1)
			printf("/dev/mem opened\n");
    }

    // Map one page of memory into user space such that the device is 
    // in that page, but it may not be at the start of the page.
    base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED,
    		memfd, dev_base & ~mask);
    if (base == (void *)-1) {
        printf("Can't map the memory to user space.\n");
        return NULL;
    }
    if (mode_verbo < 1)
		printf("Memory mapped at address %p.\n", base);
    return base + (dev_base & mask);
}

#define PL2PS_DFECTL_BASE 0xFF220000
#define PL2PS_DFECTL_SIZE 0x00100004
//#define PL2PS_DFECTL_MASK (PL2PS_DFECTL_SIZE-1)
	//Select channel
#define REG_IO_CHN_SEL 0xE020
	//mode number (4bits) + division (4bits)
#define REG_IO_MOD_SEL 0xE030
	// SYNC_ENB
#define REG_IP_SYN_CFG 0xE040
	// DMA_RECONFIG
#define REG_IP_DMA_CFG 0xE050
	// ORX_ATT-EN
#define REG_IO_ORX_CFG 0x2000
/* HW design:          chip1                  chip2
#		           GPIO[18:0]            GPIO[18:0]
#PL-io(s)
#PS-register     0xFF260000[18:0]    0xFF320000[18:0]
*/

static void udev_module_init(void)
{
    phyaddr_this_base = uspace_open_mem(PL2PS_DFECTL_BASE, PL2PS_DFECTL_SIZE);
    if (NULL == phyaddr_this_base)
        exit(1);

    phyaddr_io_mod_sel = phyaddr_this_base + REG_IO_MOD_SEL;
    phyaddr_io_orx_cfg = phyaddr_this_base + REG_IO_CHN_SEL;
    phyaddr_ip_dma_cfg = phyaddr_this_base + REG_IP_DMA_CFG;
    phyaddr_ip_syn_cfg = phyaddr_this_base + REG_IP_SYN_CFG;
}

static void udev_module_exit(void)
{
    if (NULL == phyaddr_this_base)
		return;

    printf("* Close GPIO Memory\n");
    if (-1 == munmap(phyaddr_this_base, PL2PS_DFECTL_SIZE)) {
        printf("Can't unmap memory from user space.\n");
        exit(-1);
    }
	phyaddr_this_base = NULL;

    if (0 != close(memfd)) {
        printf("Can't close /dev/mem\n");
		exit(-2);
    }
	memfd = -1;
}

/* \delta T_r and \delta T_t */
#define DELTATR 163
#define DELTATT 163
static void datapath_start(int tx_on, int rx_on)
{
    unsigned long val;

	//set ORx attenuation
    *(unsigned short *)phyaddr_io_orx_cfg |= BIT(1);

    if (tx_on) {
        *(unsigned long  *)phyaddr_ip_syn_cfg |= (DELTATR << 8) | BIT(1) | BIT(0);
		*(unsigned long  *)phyaddr_ip_dma_cfg |= BIT(31) | BIT(15);
		*(unsigned short *)phyaddr_io_mod_sel |= scfg.RFICseq;
    }

	if (rx_on) {
		*(unsigned long  *)phyaddr_ip_dma_cfg |= BIT(31) | BIT(7) | BIT(5);
	}
}

static void poll_txobssel_switch_tx(void)
{
	unsigned long old1, new1;
	unsigned long old2, new2;

	old1 = *(unsigned long  *)0xFF260000;
	old2 = *(unsigned long	*)0xFF320000;

	do {
		//chip1.tx1 <-> FE.tx1-signal-path <-> chip1.orx1
		//chip1.tx2 <-> FE.tx2-signal-path <-> chip1.orx1
		new1 = *(unsigned long  *)0xFF260000;
		if (new1 != old1) {
			if (new1 & BIT(0)) {
				*(unsigned long  *)0xFF21002C = 0x51;
			} else {
				*(unsigned long  *)0xFF21002C = 0x50;
			}
		}

		//chip2.tx1 <-> FE.tx3-signal-path <-> chip1.orx1
		//chip2.tx2 <-> FE.tx4-signal-path <-> chip1.orx1		
		new2 = *(unsigned long  *)0xFF320000;
		if (new2 != old2) {
			if (new2 & BIT(0)) {
				*(unsigned long  *)0xFF21002C = 0x53;
			} else {
				*(unsigned long  *)0xFF21002C = 0x52;
			}
		}

		/* switch one Tx path to ORx within 35us after "txObsSelect" output */
		usleep(35);
	} while (1);
}



/*
 * do bottom-half for one interupt in Linux user-space rather than kernel-space
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Jacky Wang <kenwj@sina.com>
 *
 * License: GPL, version 2.1
 *------------------------------------------------------------------------------
 * Linux ISR(Interrupt Service Routines)
 *   top-half: time-sensitive or hardware-specific staff, atmic/critical section
 *   bottom-half: 
 *------------------------------------------------------------------------------
�첽֪ͨ����
  1.�������ж���һ��static struct fasync_struct *async;
  2.��fasyncϵͳ������ע��fasync_helper(fd, filp, mode, &async);
  3.���жϷ�����򣨶��벿���װ벿�����ԣ������ź�kill_fasync(&async, SIGIO, POLL_IN);
  4.���û�Ӧ�ó�������signalע��һ����ӦSIGIO�Ļص�����signal(SIGIO, sig_handler);
  5.ͨ��fcntl(fd, F_SETOWN, getpid())��������pid�����ں�
  6.ͨ��fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | FASYNC)�����첽֪ͨ
*/
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <mach/regs-gpio.h>
#include <asm-generic/siginfo.h>
#include <linux/init.h>
#include <asm/signal.h>
#include <linux/timer.h>
#include <asm/uaccess.h>

#define DEVICE_NAME "mybeep"

void beep_start(void);
void beep_stop(void);
int  beep_irq_register(void);
unsigned int flag=1;

static struct fasync_struct *async;
struct key_irq_desc {
	unsigned int irq;
	int pin;
	int pin_setting;
	int number;
	char *name;
};

static int beep_fasync(int fd, struct file *filp, int mode)
{
	printk("application fasync!\n");
	//ע���ϲ���ý��̵���Ϣ���ϲ����fcntl����FASYNC��������ϵͳ����
	return fasync_helper(fd, filp, mode, &async);
}

static struct key_irq_desc key_irqs [] = {
	{IRQ_EINT8, S3C2410_GPG(0), S3C2410_GPG0_EINT8, 0, "KEY1"},
};

static irqreturn_t key_isr(int irq, void *dev_id)
{
	//����豸�ļ��Ľ��̷���SIGIO�ź�
	kill_fasync(&async, SIGIO, POLL_IN);
	return (IRQ_HANDLED);
}

volatile uint32_t *GPBCON;
volatile uint32_t *GPBDAT;
volatile uint32_t *GPBUP;

void beep_start(void)
{
	*GPBDAT |= (1<<0);
}

void beep_stop(void)
{
	*GPBDAT &= ~(1<<0);
}

int beep_open(struct inode *inode, struct file *filp)
{
	if (beep_irq_register() != 0) {
		printk("Request irq error!\n");
	}
	printk(KERN_ALERT "application  open!\n");
	return 0;
}

ssize_t beep_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	printk("application  read!\n");
	return 0;
}

ssize_t beep_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	printk("application  write!\n");
	return 0;
}

static int beep_release(struct inode *inode, struct file *file)
{
	disable_irq(key_irqs[0].irq);
	free_irq(key_irqs[0].irq, (void *)&key_irqs[0]);
	printk("application  close!\n");
	return beep_fasync(-1, file, 0);
}

static int beep_ioctl(struct inode *inode, struct file *file, unsigned int cmd, uint32_t arg)
{
	switch(cmd) {
	case 0:
		beep_start();
		break;
	case 1:
		beep_stop();
		break;
	default:
		break;
	}
	return 0;
}

static struct file_operations beep_ops = {
	.owner = THIS_MODULE,
	.open = beep_open,
	.release = beep_release,
	.ioctl = beep_ioctl,
	.read = beep_read,
	.write = beep_write,
	.fasync = beep_fasync,
};

static struct miscdevice beep_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = DEVICE_NAME,
	.fops = &beep_ops,
};

int beep_irq_register(void)
{
	int err;
	err = request_irq(key_irqs[0].irq, key_isr, 0, key_irqs[0].name, (void *)&key_irqs[0]);
	set_irq_type(key_irqs[0].irq, IRQ_TYPE_EDGE_RISING);
	if (err) {
		disable_irq(key_irqs[0].irq);
		free_irq(key_irqs[0].irq, (void *)&key_irqs[0]);
		return -EBUSY;
	}
	return 0;
}

static int __init beep_init(void)
{
	int ret;

	ret = misc_register(&beep_misc);
	if (ret <0) {
		printk("register miscdevice error code:%d\r\n",ret);
		return ret;
	}
	printk("beep device create!\r\n");
	GPBCON = (volatile uint32_t *)ioremap(0x56000010, 12);
	GPBDAT = GPBCON+1;
	GPBUP = GPBCON+2;
	*GPBCON &= ~((1<<0) | (1<<1));
	*GPBCON |=  (1<<0);
	*GPBUP  &= ~(1<<0);
	return 0;
}

static void __exit beep_exit(void)
{
	iounmap(GPBCON);
	misc_deregister(&beep_misc);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kingdragonfly");
module_init(beep_init);
module_exit(beep_exit);

// -----------  userland -------------
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

void sig_handler(int signo)
{
	if (signo == SIGIO) {
		printf("Receive io signal from kernel!\n");
	}
}

int main(void)
{
	int fd;

	signal(SIGIO, sig_handler);

	fd = open("/dev/mybeep", O_RDWR);
	fcntl(fd, F_SETOWN, getpid());
	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | FASYNC);

	printf("waiting key interrupt:\n");
	do {
		//sleep(1);
	} while(1);
}


/*
	Device Resource Management
--------------------------------
devm_<resource/framework>_get
	power:
		static inline struct pwm_device *devm_pwm_get(struct device *dev, const char *consumer);
		struct regulator *devm_regulator_get(struct device *dev, const char *id);
	clock:
		struct clk *devm_clk_get(struct device *dev, const char *id);
	GPIO/Reset:	control path to CPU
		int devm_gpio_request(struct device *dev, unsigned gpio, const char *label);
		static inline struct pinctrl * devm_pinctrl_get_select(struct device *dev, const char *name);
		struct reset_control *devm_reset_control_get(struct device *dev, const char *id);
	IRQ:	control path to CPU
		static inline int devm_request_irq(struct device *dev, unsigned int irq, 
			irq_handler_t handler, unsigned long irqflags, const char *devname, void *dev_id);
	memory:	kzalloc
	DMA:	fast data path without CPU
	VirtualAddressSpace:	ioremap()/request_region()
		void __iomem *devm_ioremap_resource(struct device *dev, struct resource *res);
		void __iomem *devm_ioremap(struct device *dev, resource_size_t offset, unsigned long size);

Reference:
	http://www.wowotech.net/device_model/device_resource_management.html

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

//drivers/iio/adc/ad9680.c
static int ad9680_request_fd_irqs(struct axiadc_converter *conv)
{
    struct device *dev = &conv->spi->dev;
    struct gpio_desc *gpio;

    gpio = devm_gpiod_get(dev, "fastdetect-a", GPIOD_IN);
    if (!IS_ERR(gpio)) {
        int ret, irq = gpiod_to_irq(gpio);
        if (irq < 0)
            return irq;

        ret = devm_request_threaded_irq(dev,
                irq, NULL, ad9680_fdA_handler,
                IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                "fastdetect-a", conv);
        if (ret < 0)
            return ret;
    }

    gpio = devm_gpiod_get(dev, "fastdetect-b", GPIOD_IN);
    if (!IS_ERR(gpio)) {
        int ret, irq = gpiod_to_irq(gpio);
        if (irq < 0)
            return irq;

        ret = devm_request_threaded_irq(dev,
                irq, NULL, ad9680_fdB_handler,
                IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                "fastdetect-b", conv);
        if (ret < 0)  
            return ret;
    }

    return 0;
}

static int ad9680_probe(struct spi_device *spi)
{
	//...
	ret = ad9680_request_fd_irqs(conv);
	//...
}

*/

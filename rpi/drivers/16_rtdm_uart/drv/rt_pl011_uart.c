
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sizes.h>
#include <linux/io.h>
#include <linux/acpi.h>

#include <rtdm/serial.h>
#include <rtdm/driver.h>
#include <rtdm/rtdm.h>

#include "amba-pl011.h"

#define UART_NR			14

#define SERIAL_AMBA_MAJOR	204
#define SERIAL_AMBA_MINOR	64
#define SERIAL_AMBA_NR		UART_NR

#define AMBA_ISR_PASS_LIMIT	256

#define UART_DR_ERROR		(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)


#define TX_FIFO_SIZE		32
#define IN_BUFFER_SIZE		4096
#define OUT_BUFFER_SIZE		4096


#define RT_PL011_UART_MAX	5

static int tx_fifo[RT_PL011_UART_MAX];
module_param_array(tx_fifo, int, NULL, 0400);
MODULE_PARM_DESC(tx_fifo, "Transmitter FIFO size");

static u16 pl011_std_offsets[REG_ARRAY_SIZE] = {
	[REG_DR] = UART01x_DR,
	[REG_FR] = UART01x_FR,
	[REG_LCRH_RX] = UART011_LCRH,
	[REG_LCRH_TX] = UART011_LCRH,
	[REG_IBRD] = UART011_IBRD,
	[REG_FBRD] = UART011_FBRD,
	[REG_CR] = UART011_CR,
	[REG_IFLS] = UART011_IFLS,
	[REG_IMSC] = UART011_IMSC,
	[REG_RIS] = UART011_RIS,
	[REG_MIS] = UART011_MIS,
	[REG_ICR] = UART011_ICR,
	[REG_DMACR] = UART011_DMACR,
};

/* There is by now at least one vendor with differing details, so handle it */
struct vendor_data {
	const u16		*reg_offset;
	unsigned int		ifls;
	unsigned int		fr_busy;
	unsigned int		fr_dsr;
	unsigned int		fr_cts;
	unsigned int		fr_ri;
	unsigned int		inv_fr;
	bool			access_32b;
	bool			oversampling;
	bool			dma_threshold;
	bool			cts_event_workaround;
	bool			always_enabled;
	bool			fixed_options;

	unsigned int (*get_fifosize)(struct amba_device *dev);
};

static unsigned int get_fifosize_arm(struct amba_device *dev)
{
	return amba_rev(dev) < 3 ? 16 : 32;
}

static struct vendor_data vendor_arm = {
	.reg_offset		= pl011_std_offsets,
	.ifls			= UART011_IFLS_RX4_8|UART011_IFLS_TX4_8,
	.fr_busy		= UART01x_FR_BUSY,
	.fr_dsr			= UART01x_FR_DSR,
	.fr_cts			= UART01x_FR_CTS,
	.fr_ri			= UART011_FR_RI,
	.oversampling		= false,
	.dma_threshold		= false,
	.cts_event_workaround	= false,
	.always_enabled		= false,
	.fixed_options		= false,
	.get_fifosize		= get_fifosize_arm,
};



static u16 pl011_st_offsets[REG_ARRAY_SIZE] = {
	[REG_DR] = UART01x_DR,
	[REG_ST_DMAWM] = ST_UART011_DMAWM,
	[REG_ST_TIMEOUT] = ST_UART011_TIMEOUT,
	[REG_FR] = UART01x_FR,
	[REG_LCRH_RX] = ST_UART011_LCRH_RX,
	[REG_LCRH_TX] = ST_UART011_LCRH_TX,
	[REG_IBRD] = UART011_IBRD,
	[REG_FBRD] = UART011_FBRD,
	[REG_CR] = UART011_CR,
	[REG_IFLS] = UART011_IFLS,
	[REG_IMSC] = UART011_IMSC,
	[REG_RIS] = UART011_RIS,
	[REG_MIS] = UART011_MIS,
	[REG_ICR] = UART011_ICR,
	[REG_DMACR] = UART011_DMACR,
	[REG_ST_XFCR] = ST_UART011_XFCR,
	[REG_ST_XON1] = ST_UART011_XON1,
	[REG_ST_XON2] = ST_UART011_XON2,
	[REG_ST_XOFF1] = ST_UART011_XOFF1,
	[REG_ST_XOFF2] = ST_UART011_XOFF2,
	[REG_ST_ITCR] = ST_UART011_ITCR,
	[REG_ST_ITIP] = ST_UART011_ITIP,
	[REG_ST_ABCR] = ST_UART011_ABCR,
	[REG_ST_ABIMSC] = ST_UART011_ABIMSC,
};

static unsigned int get_fifosize_st(struct amba_device *dev)
{
	return 64;
}

static struct vendor_data vendor_st = {
	.reg_offset		= pl011_st_offsets,
	.ifls			= UART011_IFLS_RX_HALF|UART011_IFLS_TX_HALF,
	.fr_busy		= UART01x_FR_BUSY,
	.fr_dsr			= UART01x_FR_DSR,
	.fr_cts			= UART01x_FR_CTS,
	.fr_ri			= UART011_FR_RI,
	.oversampling		= true,
	.dma_threshold		= true,
	.cts_event_workaround	= true,
	.always_enabled		= false,
	.fixed_options		= false,
	.get_fifosize		= get_fifosize_st,
};

static const u16 pl011_zte_offsets[REG_ARRAY_SIZE] = {
	[REG_DR] = ZX_UART011_DR,
	[REG_FR] = ZX_UART011_FR,
	[REG_LCRH_RX] = ZX_UART011_LCRH,
	[REG_LCRH_TX] = ZX_UART011_LCRH,
	[REG_IBRD] = ZX_UART011_IBRD,
	[REG_FBRD] = ZX_UART011_FBRD,
	[REG_CR] = ZX_UART011_CR,
	[REG_IFLS] = ZX_UART011_IFLS,
	[REG_IMSC] = ZX_UART011_IMSC,
	[REG_RIS] = ZX_UART011_RIS,
	[REG_MIS] = ZX_UART011_MIS,
	[REG_ICR] = ZX_UART011_ICR,
	[REG_DMACR] = ZX_UART011_DMACR,
};

static unsigned int get_fifosize_zte(struct amba_device *dev)
{
	return 16;
}

static struct vendor_data vendor_zte = {
	.reg_offset		= pl011_zte_offsets,
	.access_32b		= true,
	.ifls			= UART011_IFLS_RX4_8|UART011_IFLS_TX4_8,
	.fr_busy		= ZX_UART01x_FR_BUSY,
	.fr_dsr			= ZX_UART01x_FR_DSR,
	.fr_cts			= ZX_UART01x_FR_CTS,
	.fr_ri			= ZX_UART011_FR_RI,
	.get_fifosize		= get_fifosize_zte,
};


struct rt_uart_amba_port {

	unsigned char __iomem *membase;	/* read/write[bwl] */
	resource_size_t mapbase;	/* for ioremap */
	unsigned int irq;		/* irq number */
	int tx_fifo;
	
	unsigned int have_rtscts;
	unsigned int use_dcedte;
	unsigned int use_hwflow;
	
	const u16		*reg_offset;
	struct clk		*clk;
	const struct vendor_data *vendor;
	unsigned int		dmacr;		/* dma control reg */
	unsigned int		im;		/* interrupt mask */
	unsigned int		old_status;
	unsigned int		fifosize;	/* vendor-specific */
	unsigned int		old_cr;		/* state during shutdown */
	unsigned int		fixed_baud;	/* vendor-set fixed baud rate */
	char			type[12];
	
	unsigned char iotype;
	
	struct rtdm_device rtdm_dev;
};

struct rt_imx_uart_ctx {
	struct rtser_config config;	/* current device configuration */

	rtdm_irq_t irq_handle;		/* device IRQ handle */
	rtdm_lock_t lock;		/* lock to protect context struct */

	int in_head;			/* RX ring buffer, head pointer */
	int in_tail;			/* RX ring buffer, tail pointer */
	size_t in_npend;		/* pending bytes in RX ring */
	int in_nwait;			/* bytes the user waits for */
	rtdm_event_t in_event;		/* raised to unblock reader */
	char in_buf[IN_BUFFER_SIZE];	/* RX ring buffer */

	volatile unsigned long in_lock;	/* single-reader lock */
	uint64_t *in_history;		/* RX timestamp buffer */

	int out_head;			/* TX ring buffer, head pointer */
	int out_tail;			/* TX ring buffer, tail pointer */
	size_t out_npend;		/* pending bytes in TX ring */
	rtdm_event_t out_event;		/* raised to unblock writer */
	char out_buf[OUT_BUFFER_SIZE];	/* TX ring buffer */
	rtdm_mutex_t out_lock;		/* single-writer mutex */

	uint64_t last_timestamp;	/* timestamp of last event */
	int ioc_events;			/* recorded events */
	rtdm_event_t ioc_event;		/* raised to unblock event waiter */
	volatile unsigned long ioc_event_lock;	/* single-waiter lock */

	int ier_status;			/* IER cache */
	int mcr_status;			/* MCR cache */
	int status;			/* cache for LSR + soft-states */
	int saved_errors;		/* error cache for RTIOC_GET_STATUS */

	/*
	 * The port structure holds all the information about the UART
	 * port like base address, and so on.
	 */
	struct rt_uart_amba_port *port;
};

static struct rt_uart_amba_port *amba_ports[UART_NR];


static unsigned int pl011_reg_to_offset(const struct rt_uart_amba_port *uap,
	unsigned int reg)
{
	return uap->reg_offset[reg];
}

static unsigned int pl011_read(const struct rt_uart_amba_port *uap,
	unsigned int reg)
{
	void __iomem *addr = uap->membase + pl011_reg_to_offset(uap, reg);

	return (uap->iotype == UPIO_MEM32) ?
		readl_relaxed(addr) : readw_relaxed(addr);
}

static void pl011_write(unsigned int val, const struct rt_uart_amba_port *uap,
	unsigned int reg)
{
	void __iomem *addr = uap->membase + pl011_reg_to_offset(uap, reg);

	if (uap->iotype == UPIO_MEM32)
		writel_relaxed(val, addr);
	else
		writew_relaxed(val, addr);
}


/* unregisters the driver also if no more ports are left */
static void pl011_unregister_port(struct rt_uart_amba_port *uap)
{
	int i;
	bool busy = false;

	for (i = 0; i < ARRAY_SIZE(amba_ports); i++) {
		if (amba_ports[i] == uap)
			amba_ports[i] = NULL;
		else if (amba_ports[i])
			busy = true;
	}
//TBD	pl011_dma_remove(uap);
	//if (!busy)
	//	uart_unregister_driver(&amba_reg);
}

static int pl011_find_free_port(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(amba_ports); i++)
		if (amba_ports[i] == NULL)
			return i;

	return -EBUSY;
}

static int pl011_setup_port(struct device *dev, struct rt_uart_amba_port *uap,
			    struct resource *mmiobase, int index)
{
	void __iomem *base;
	
	//TBD - dev: struct rtdm_device
	base = devm_ioremap_resource(dev, mmiobase);
	if (IS_ERR(base))
		return PTR_ERR(base);

	/* Don't use DT serial<n> aliases - it causes the device to
	   be renumbered to ttyAMA1 if it is the second serial port in the
	   system, even though the other one is ttyS0. The 8250 driver
	   doesn't use this logic, so always remains ttyS0.
	index = pl011_probe_dt_alias(index, dev);
	*/

	uap->old_cr = 0;
	//uap->port.dev = dev;
	uap->mapbase = mmiobase->start;
	uap->membase = base;
	//uap->port.fifosize = uap->fifosize;
	//uap->port.has_sysrq = IS_ENABLED(CONFIG_SERIAL_AMBA_PL011_CONSOLE);
	//uap->port.flags = UPF_BOOT_AUTOCONF;
	//uap->port.line = index;

	amba_ports[index] = uap;

	return 0;
}

static int pl011_register_port(struct rt_uart_amba_port *uap)
{
	int ret=0;

	/* Ensure interrupts from this UART are masked and cleared */
	pl011_write(0, uap, REG_IMSC);
	pl011_write(0xffff, uap, REG_ICR);

	//if (!amba_reg.state) {
	//	ret = uart_register_driver(&amba_reg);
	//	if (ret < 0) {
	//		dev_err(uap->port.dev,
	//			"Failed to register AMBA-PL011 driver\n");
	//		for (i = 0; i < ARRAY_SIZE(amba_ports); i++)
	//			if (amba_ports[i] == uap)
	//				amba_ports[i] = NULL;
	//		return ret;
	//	}
	//}
	//
	//ret = uart_add_one_port(&amba_reg, &uap->port);
	//if (ret)
	//	pl011_unregister_port(uap);

	return ret;
}



static int rt_pl011_uart_open(struct rtdm_fd *fd, int oflags)
{
	printk("%s", __FUNCTION__);
	
	return 0;
}

void rt_pl011_uart_close(struct rtdm_fd *fd)
{
	printk("%s", __FUNCTION__);
	
	
}


static int rt_pl011_uart_ioctl(struct rtdm_fd *fd,
			     unsigned int request, void *arg)
{
	int err = 0;
	printk("%s", __FUNCTION__);
	
	return err;
}

ssize_t rt_pl011_uart_read(struct rtdm_fd *fd, void *buf, size_t nbyte)
{
	ssize_t ret = -EAGAIN;	/* for non-blocking read */
	printk("%s", __FUNCTION__);
	
	return ret;
}

static ssize_t rt_pl011_uart_write(struct rtdm_fd *fd, const void *buf,
				size_t nbyte)
{
	int ret,err = 0;
	
	printk("%s", __FUNCTION__);
	return ret;
}

static struct rtdm_driver pl011_uart_driver = {
	.profile_info		= RTDM_PROFILE_INFO(imx_uart,
						    RTDM_CLASS_SERIAL,
						    RTDM_SUBCLASS_16550A,
						    RTSER_PROFILE_VER),
	.device_count		= RT_PL011_UART_MAX,
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.context_size		= sizeof(struct rt_imx_uart_ctx),
	.ops = {
		.open		= rt_pl011_uart_open,
		.close		= rt_pl011_uart_close,
		.ioctl_rt	= rt_pl011_uart_ioctl,
		.ioctl_nrt	= rt_pl011_uart_ioctl,
		.read_rt	= rt_pl011_uart_read,
		.write_rt	= rt_pl011_uart_write,
	},
};

static int rt_pl011_probe(struct amba_device *dev, const struct amba_id *id)
{
	struct rtdm_device *rtdm_dev;
	struct rt_uart_amba_port *uap;
	struct vendor_data *vendor = id->data;
	int portnr, ret;
	printk("%s:%d", __FUNCTION__, __LINE__);

	portnr = pl011_find_free_port();
	if (portnr < 0)
		return portnr;

	uap = devm_kzalloc(&dev->dev, sizeof(struct rt_uart_amba_port),
			   GFP_KERNEL);
	if (!uap)
		return -ENOMEM;

	uap->clk = devm_clk_get(&dev->dev, NULL);
	if (IS_ERR(uap->clk))
		return PTR_ERR(uap->clk);

	if (of_property_read_bool(dev->dev.of_node, "cts-event-workaround")) {
	    vendor->cts_event_workaround = true;
	    dev_info(&dev->dev, "cts_event_workaround enabled\n");
	}

	uap->reg_offset = vendor->reg_offset;
	uap->vendor = vendor;
	uap->fifosize = vendor->get_fifosize(dev);
	uap->iotype = vendor->access_32b ? UPIO_MEM32 : UPIO_MEM;
	uap->irq = dev->irq[0];
	// uap->port.ops = &amba_pl011_pops;

	snprintf(uap->type, sizeof(uap->type), "PL011 rev%u", amba_rev(dev));
	amba_set_drvdata(dev, uap);

	ret = pl011_setup_port(&dev->dev, uap, &dev->res, portnr);
	if (ret)
		return ret;
	
	ret = pl011_register_port(uap);
	if (ret)
		return ret;
	
	rtdm_dev = &uap->rtdm_dev;
	rtdm_dev->driver = &pl011_uart_driver;
	rtdm_dev->label = "rtser%d";
	rtdm_dev->device_data = uap;
	
	if (!tx_fifo[id->id] || tx_fifo[id->id] > TX_FIFO_SIZE)
		uap->tx_fifo = TX_FIFO_SIZE;
	else
		uap->tx_fifo = tx_fifo[id->id];
	
	printk("%s:%d", __FUNCTION__, __LINE__);
	// ??? enable clk
	uap->use_hwflow = 1;
	
	ret = rtdm_dev_register(rtdm_dev);
	if (ret)
		return ret;

	amba_set_drvdata(dev, uap);
	
	pr_info("%s on IMX UART%d: membase=0x%p irq=%d clk=%d\n",
	       rtdm_dev->name, id->id, uap->membase, uap->irq, uap->clk);

	return 0;
}

static void rt_pl011_remove(struct amba_device *dev)
{
	struct rt_uart_amba_port *uap = amba_get_drvdata(dev);
	pl011_unregister_port(uap);
	
}



static const struct amba_id pl011_ids[] = {
	{
		.id	= 0x00041011,
		.mask	= 0x000fffff,
		.data	= &vendor_arm,
	},
	{
		.id	= 0x00380802,
		.mask	= 0x00ffffff,
		.data	= &vendor_st,
	},
	{
		.id	= AMBA_LINUX_ID(0x00, 0x1, 0xffe),
		.mask	= 0x00ffffff,
		.data	= &vendor_zte,
	},
	{ 0, 0 },
};

MODULE_DEVICE_TABLE(amba, pl011_ids);

static struct amba_driver rt_pl011_uart_driver = {
	.drv = {
		.name	= "rt-uart-pl011",
		//.pm	= &pl011_dev_pm_ops,
		.suppress_bind_attrs = IS_BUILTIN(CONFIG_SERIAL_AMBA_PL011),
	},
	.id_table	= pl011_ids,
	.probe		= rt_pl011_probe,
	.remove		= rt_pl011_remove,
};


static int __init rt_pl011_uart_init(void)
{
	int ret;

	if (!rtdm_available())
		return -ENODEV;

	//ret = platform_driver_register(&rt_pl011_uart_driver);
    ret = amba_driver_register(&rt_pl011_uart_driver);
	if (ret) {
		pr_err("%s; Could not register  driver (err=%d)\n",
			__func__, ret);
	}

	return ret;
}

static void __exit rt_pl011_uart_exit(void)
{
	//platform_driver_unregister(&rt_pl011_uart_driver);
    amba_driver_unregister(&rt_pl011_uart_driver);
}

module_init(rt_pl011_uart_init);
module_exit(rt_pl011_uart_exit);

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

MODULE_AUTHOR("Will Chen <Will8.Chen@advantech.com.tw>");
MODULE_DESCRIPTION("RTDM-based driver for BCM PL011 UARTs");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");

#define UART_NR			14

#define SERIAL_AMBA_MAJOR	204
#define SERIAL_AMBA_MINOR	64
#define SERIAL_AMBA_NR		UART_NR

#define AMBA_ISR_PASS_LIMIT	256

#define UART_DR_ERROR		(UART011_DR_OE|UART011_DR_BE|UART011_DR_PE|UART011_DR_FE)
#define UART_DUMMY_DR_RX	(1 << 16)


#define IN_BUFFER_SIZE		4096
#define OUT_BUFFER_SIZE		4096

#define TX_FIFO_SIZE		32

#define PARITY_MASK		0x03
#define DATA_BITS_MASK		0x03
#define STOP_BITS_MASK		0x01
#define FIFO_MASK		0xC0
#define EVENT_MASK		0x0F

#define IER_RX			0x01
#define IER_TX			0x02
#define IER_STAT		0x04
#define IER_MODEM		0x08


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


struct rt_amba_uart_port {
    
    struct device *dev;
    
	unsigned char __iomem *membase;	/* read/write[bwl] */
	resource_size_t mapbase;	/* for ioremap */
	unsigned int irq;		/* irq number */
	int tx_fifo;
	
	unsigned int have_rtscts;
	unsigned int use_dcedte;
	unsigned int use_hwflow;
	
    unsigned int status;
    
	const u16		*reg_offset;
	struct clk		*clk;
    unsigned int    uartclk;
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

struct rt_amba_uart_ctx {
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
	struct rt_amba_uart_port *port;
};

static struct rt_amba_uart_port *amba_ports[UART_NR];


static unsigned int pl011_reg_to_offset(const struct rt_amba_uart_port *uap,
	unsigned int reg)
{
	return uap->reg_offset[reg];
}

static unsigned int pl011_read(const struct rt_amba_uart_port *uap,
	unsigned int reg)
{
	void __iomem *addr = uap->membase + pl011_reg_to_offset(uap, reg);

	return (uap->iotype == UPIO_MEM32) ?
		readl_relaxed(addr) : readw_relaxed(addr);
}

static void pl011_write(unsigned int val, const struct rt_amba_uart_port *uap,
	unsigned int reg)
{
	void __iomem *addr = uap->membase + pl011_reg_to_offset(uap, reg);
    
    pr_info("Writing to reg[%d], value: 0x%x\n", reg, val);
    
	if (uap->iotype == UPIO_MEM32)
		writel_relaxed(val, addr);
	else
		writew_relaxed(val, addr);
}


static int pl011_setup_port(struct device *dev, struct rt_amba_uart_port *uap,
			    struct resource *mmiobase, int index)
{
	void __iomem *base;
	
	base = devm_ioremap_resource(dev, mmiobase);
	if (IS_ERR(base))
		return PTR_ERR(base);
    
	/* Don't use DT serial<n> aliases - it causes the device to
	   be renumbered to ttyAMA1 if it is the second serial port in the
	   system, even though the other one is ttyS0. The 8250 driver
	   doesn't use this logic, so always remains ttyS0.
	   index = pl011_probe_dt_alias(index, dev);
	*/
    
    printk("uart[%d] mapbase:phy_addr=0x%08llx", index, mmiobase->start);
	uap->old_cr = 0;
	uap->dev = dev;
	uap->mapbase = mmiobase->start;
	uap->membase = base;
	//uap->port.fifosize = uap->fifosize;
	//uap->port.has_sysrq = IS_ENABLED(CONFIG_SERIAL_AMBA_PL011_CONSOLE);
	//uap->port.flags = UPF_BOOT_AUTOCONF;
	//uap->port.line = index;

	amba_ports[index] = uap;
	return 0;
}

static int pl011_register_port(struct rt_amba_uart_port *uap)
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

/* unregisters the driver also if no more ports are left */
static void pl011_unregister_port(struct rt_amba_uart_port *uap)
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




static int pl011_hwinit(struct rt_amba_uart_port *port)
{
	int retval;

	/* Optionaly enable pins to be muxed in and configured */
	pinctrl_pm_select_default_state(port->dev);

	/*
	 * Try to enable the clock producer.
	 */
	retval = clk_prepare_enable(port->clk);
	if (retval)
		return retval;
    
	port->uartclk = clk_get_rate(port->clk);

	/* Clear pending error and receive interrupts */
	pl011_write(UART011_OEIS | UART011_BEIS | UART011_PEIS |
		    UART011_FEIS | UART011_RTIS | UART011_RXIS,
		    port, REG_ICR);

	/*
	 * Save interrupts enable mask, and enable RX interrupts in case if
	 * the interrupt is used for NMI entry.
	 */
	port->im = pl011_read(port, REG_IMSC);
	pl011_write(UART011_RTIM | UART011_RXIM, port, REG_IMSC);

	if (dev_get_platdata(port->dev)) {
		struct amba_pl011_data *plat;

		plat = dev_get_platdata(port->dev);
		if (plat->init)
			plat->init();
	}
	return 0;
}

static bool pl011_split_lcrh(const struct rt_amba_uart_port *uap)
{
	return pl011_reg_to_offset(uap, REG_LCRH_RX) !=
	       pl011_reg_to_offset(uap, REG_LCRH_TX);
}

static void pl011_write_lcr_h(struct rt_amba_uart_port *uap, unsigned int lcr_h)
{
	pl011_write(lcr_h, uap, REG_LCRH_RX);
	if (pl011_split_lcrh(uap)) {
		int i;
		/*
		 * Wait 10 PCLKs before writing LCRH_TX register,
		 * to get this delay write read only register 10 times
		 */
		for (i = 0; i < 10; ++i)
			pl011_write(0xff, uap, REG_MIS);
		pl011_write(lcr_h, uap, REG_LCRH_TX);
	}
}

/*
 * Enable interrupts, only timeouts when using DMA
 * if initial RX DMA job failed, start in interrupt mode
 * as well.
 */
static void pl011_enable_interrupts(struct rt_amba_uart_ctx *ctx)
{
    struct rt_amba_uart_port *uap = ctx->port;
    rtdm_lockctx_t lock_ctx;
	unsigned int i;

    rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	/* Clear out any spuriously appearing RX interrupts */
	pl011_write(UART011_RTIS | UART011_RXIS, uap, REG_ICR);

	/*
	 * RXIS is asserted only when the RX FIFO transitions from below
	 * to above the trigger threshold.  If the RX FIFO is already
	 * full to the threshold this can't happen and RXIS will now be
	 * stuck off.  Drain the RX FIFO explicitly to fix this:
	 */
	//TBD - fifosize
	for (i = 0; i < uap->fifosize * 2; ++i) {
		if (pl011_read(uap, REG_FR) & UART01x_FR_RXFE)
			break;

		pl011_read(uap, REG_DR);
	}

	uap->im = UART011_RTIM | UART011_RXIM;
	pl011_write(uap->im, uap, REG_IMSC);
    rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
}

static void pl011_disable_interrupts(struct rt_amba_uart_ctx *ctx)
{
    struct rt_amba_uart_port *uap = ctx->port;
    rtdm_lock_get(&ctx->lock);
    
	/* mask all interrupts and clear all pending ones */
	uap->im = 0;
	pl011_write(uap->im, uap, REG_IMSC);
	pl011_write(0xffff, uap, REG_ICR);
    
    rtdm_lock_put(&ctx->lock);
}

static void pl011_shutdown_channel(struct rt_amba_uart_port *uap,
					unsigned int lcrh)
{
      unsigned long val;

      val = pl011_read(uap, lcrh);
      val &= ~(UART01x_LCRH_BRK | UART01x_LCRH_FEN);
      pl011_write(val, uap, lcrh);
}

/*
 * disable the port. It should not disable RTS and DTR.
 * Also RTS and DTR state should be preserved to restore
 * it during startup().
 */
static void pl011_disable_uart(struct rt_amba_uart_ctx *ctx)
{
    struct rt_amba_uart_port *uap = ctx->port;
    rtdm_lockctx_t lock_ctx;
	unsigned int cr;
	
	ctx->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx); //TBD
	cr = pl011_read(uap, REG_CR);
	uap->old_cr = cr;
	cr &= UART011_CR_RTS | UART011_CR_DTR;
	cr |= UART01x_CR_UARTEN | UART011_CR_TXE;
	pl011_write(cr, uap, REG_CR);
	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx); //TBD

	/*
	 * disable break condition and fifos
	 */
	pl011_shutdown_channel(uap, REG_LCRH_RX);
	if (pl011_split_lcrh(uap))
		pl011_shutdown_channel(uap, REG_LCRH_TX);
}

static void rt_pl011_tx_chars(struct rt_amba_uart_ctx *ctx);
static void rt_amba_uart_start_tx(struct rt_amba_uart_ctx *ctx)
{
	struct rt_amba_uart_port *uap = ctx->port;
    //rtdm_printk("%s\n", __FUNCTION__);
    rt_pl011_tx_chars(ctx);
    uap->im |= UART011_TXIM;
    pl011_write(uap->im, uap, REG_IMSC);
}

static void rt_amba_uart_stop_tx(struct rt_amba_uart_ctx *ctx)
{
	struct rt_amba_uart_port *uap = ctx->port;
	uap->im &= ~UART011_TXIM;
	pl011_write(uap->im, uap, REG_IMSC);
}


static void rt_pl011_enable_ms(struct rt_amba_uart_ctx *ctx)
{
	struct rt_amba_uart_port *uap = ctx->port;

	uap->im |= UART011_RIMIM|UART011_CTSMIM|UART011_DCDMIM|UART011_DSRMIM;
	pl011_write(uap->im, uap, REG_IMSC);
}


static int rt_pl011_rx_chars(struct rt_amba_uart_ctx *ctx,
				uint64_t *timestamp)
{
    struct rt_amba_uart_port *uap = ctx->port;
    unsigned int rx;
	int rbytes = 0;
	int lsr = 0;
    
    while(!(pl011_read(uap, REG_FR) & UART01x_FR_RXFE)){
        rx = pl011_read(uap, REG_DR) | UART_DUMMY_DR_RX;
                
        if (unlikely(rx & UART_DR_ERROR)) {
			if (rx & UART011_DR_PE)
				lsr |= RTSER_LSR_PARITY_ERR;
			else if (rx & UART011_DR_FE)
				lsr |= RTSER_LSR_FRAMING_ERR;
            if (rx & UART011_DR_OE)
				lsr |= RTSER_LSR_OVERRUN_ERR;
        }
        
        /* save received character */
		ctx->in_buf[ctx->in_tail] = rx & 0xff;
		if (ctx->in_history)
			ctx->in_history[ctx->in_tail] = *timestamp;
		ctx->in_tail = (ctx->in_tail + 1) & (IN_BUFFER_SIZE - 1);

		if (unlikely(ctx->in_npend >= IN_BUFFER_SIZE))
			lsr |= RTSER_SOFT_OVERRUN_ERR;
		else
			ctx->in_npend++;

		rbytes++;
    }
    
	/* save new errors */
	ctx->status |= lsr;

	return rbytes;
}


static void rt_pl011_tx_chars(struct rt_amba_uart_ctx *ctx)
{
    struct rt_amba_uart_port *uap = ctx->port;
    unsigned int ch;
    
    rtdm_printk("out_npend=%d\n", (int)ctx->out_npend);
	while (ctx->out_npend > 0 &&
	       !(pl011_read(uap, REG_FR) & UART01x_FR_TXFF)) {
        
        ch = ctx->out_buf[ctx->out_head++];
        rtdm_printk("0x%02x ", ch);
        pl011_write(ch, uap, REG_DR);
		ctx->out_head &= (OUT_BUFFER_SIZE - 1);
		ctx->out_npend--;
	}
}

static int pl011_modem_status(struct rt_amba_uart_port *uap)
{
	unsigned int status, delta, events=0;

	status = pl011_read(uap, REG_FR) & UART01x_FR_MODEM_ANY;

	delta = status ^ uap->old_status;
	uap->old_status = status;

	if (!delta)
		return events;
	
	//TBD??? - 放到 set_config 
	if (delta & UART01x_FR_DCD){
		events |= (status & UART01x_FR_DCD)? RTSER_EVENT_MODEMHI:RTSER_EVENT_MODEMLO;
		//uart_handle_dcd_change(&uap, status & UART01x_FR_DCD); //TBD
	}
	
	//not used in this chip
	//if (delta & uap->vendor->fr_dsr)
	//	uap->port.icount.dsr++;
	
	if (delta & uap->vendor->fr_cts){
		events |= (status & uap->vendor->fr_cts)? RTSER_EVENT_MODEMHI:RTSER_EVENT_MODEMLO;
		//uart_handle_cts_change(&uap, status & uap->vendor->fr_cts); //TBD
	}
    
    return events;
}

static void check_apply_cts_event_workaround(struct rt_amba_uart_port *uap)
{
	if (!uap->vendor->cts_event_workaround)
		return;

	/* workaround to make sure that all bits are unlocked.. */
	pl011_write(0x00, uap, REG_ICR);

	/*
	 * WA: introduce 26ns(1 uart clk) delay before W1C;
	 * single apb access will incur 2 pclk(133.12Mhz) delay,
	 * so add 2 dummy reads
	 */
	pl011_read(uap, REG_ICR);
	pl011_read(uap, REG_ICR);
}

static int rt_pl011_int(rtdm_irq_t *irq_context)
{
    uint64_t timestamp = rtdm_clock_read();
    struct rt_amba_uart_ctx *ctx;
    struct rt_amba_uart_port *uap;
    int rbytes = 0, events = 0;
    unsigned int status, pass_counter = AMBA_ISR_PASS_LIMIT;
    int ret = RTDM_IRQ_NONE;
    
    rtdm_printk("%s", __func__);
    ctx = rtdm_irq_get_arg(irq_context, struct rt_amba_uart_ctx);
    uap = ctx->port;
    rtdm_lock_get(&ctx->lock);
    
	status = pl011_read(uap, REG_RIS) & uap->im;
	if (status) {
		do {
			check_apply_cts_event_workaround(uap);

			pl011_write(status & ~(UART011_TXIS|UART011_RTIS|
					       UART011_RXIS), uap, REG_ICR);

			if (status & (UART011_RTIS|UART011_RXIS)) {
                rt_pl011_rx_chars(ctx, &timestamp);
				events |= RTSER_EVENT_RXPEND;
			}
			
			if (status & (UART011_DSRMIS|UART011_DCDMIS|
				      UART011_CTSMIS|UART011_RIMIS)){
				events |= pl011_modem_status(uap);
			}
			
			if (status & UART011_TXIS){
				rt_pl011_tx_chars(ctx);
			}

			if (pass_counter-- == 0)
				break;

			status = pl011_read(uap, REG_RIS) & uap->im;
		} while (status != 0);
		
		ret = RTDM_IRQ_HANDLED;
	}
	
	if (ctx->in_nwait > 0) {
		if ((ctx->in_nwait <= rbytes) || ctx->status) {
			ctx->in_nwait = 0;
			rtdm_event_signal(&ctx->in_event);
		} else {
			ctx->in_nwait -= rbytes;
		}
	}

	if (ctx->status) {
		events |= RTSER_EVENT_ERRPEND;
		ctx->ier_status &= ~IER_STAT;
	}

	if (events & ctx->config.event_mask) {
		int old_events = ctx->ioc_events;

		ctx->last_timestamp = timestamp;
		ctx->ioc_events = events;

		if (!old_events)
			rtdm_event_signal(&ctx->ioc_event);
	}

	if ((ctx->ier_status & IER_TX) && (ctx->out_npend == 0)) {
		rt_amba_uart_stop_tx(ctx);
		
		ctx->ier_status &= ~IER_TX;
		rtdm_event_signal(&ctx->out_event);
	}
	
	rtdm_lock_put(&ctx->lock);
	
	if (ret != RTDM_IRQ_HANDLED)
		pr_warn("%s: unhandled interrupt\n", __func__);
	return ret;
}

static unsigned int rt_amba_uart_get_msr(struct rt_amba_uart_ctx *ctx)
{
	struct rt_amba_uart_port *uap = ctx->port;
	unsigned int result = 0;
	unsigned int status = pl011_read(uap, REG_FR);

#define TIOCMBIT(uartbit, tiocmbit)	\
	if (status & uartbit)		\
		result |= tiocmbit

	TIOCMBIT(UART01x_FR_DCD, TIOCM_CAR);
	TIOCMBIT(uap->vendor->fr_dsr, TIOCM_DSR);
	TIOCMBIT(uap->vendor->fr_cts, TIOCM_CTS);
	TIOCMBIT(uap->vendor->fr_ri, TIOCM_RNG);
#undef TIOCMBIT
	return result;
}

static void rt_amba_uart_set_mcr(struct rt_amba_uart_ctx *ctx,
				unsigned int mcr)
{
	struct rt_amba_uart_port *uap = ctx->port;
	unsigned int cr;

	cr = pl011_read(uap, REG_CR);

#define	TIOCMBIT(tiocmbit, uartbit)		\
	if (mcr & tiocmbit)		\
		cr |= uartbit;		\
	else				\
		cr &= ~uartbit

	TIOCMBIT(TIOCM_RTS, UART011_CR_RTS);
	TIOCMBIT(TIOCM_DTR, UART011_CR_DTR);
	TIOCMBIT(TIOCM_OUT1, UART011_CR_OUT1);
	TIOCMBIT(TIOCM_OUT2, UART011_CR_OUT2);
	TIOCMBIT(TIOCM_LOOP, UART011_CR_LBE);

	if (ctx->status & UPSTAT_AUTORTS) {
		/* We need to disable auto-RTS if we want to turn RTS off */
		TIOCMBIT(TIOCM_RTS, UART011_CR_RTSEN);
	}
#undef TIOCMBIT

	pl011_write(cr, uap, REG_CR);
}

static void rt_amba_uart_break_ctl(struct rt_amba_uart_ctx *ctx,
				  int break_state)
{
    struct rt_amba_uart_port *uap = ctx->port;
    unsigned int lcr_h;
    lcr_h = pl011_read(uap, REG_LCRH_RX);

	if (break_state == RTSER_BREAK_SET)
		lcr_h |= UART01x_LCRH_BRK;
	else
		lcr_h &= ~UART01x_LCRH_BRK;
    
    pl011_write_lcr_h(uap, lcr_h);
}

static int rt_amba_uart_set_config(struct rt_amba_uart_ctx *ctx,
				  const struct rtser_config *config,
				  uint64_t **in_history_ptr)
{
	struct rt_amba_uart_port *uap = ctx->port;
	rtdm_lockctx_t lock_ctx;
	int err = 0;

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
    
	if (config->config_mask & RTSER_SET_BAUD)
		ctx->config.baud_rate = config->baud_rate;
	if (config->config_mask & RTSER_SET_DATA_BITS)
		ctx->config.data_bits = config->data_bits & DATA_BITS_MASK;
	if (config->config_mask & RTSER_SET_PARITY)
		ctx->config.parity = config->parity & PARITY_MASK;
	if (config->config_mask & RTSER_SET_STOP_BITS)
		ctx->config.stop_bits = config->stop_bits & STOP_BITS_MASK;
    
    /* Timeout manipulation is not atomic. The user is supposed to take
	 * care not to use and change timeouts at the same time.
	 */
	if (config->config_mask & RTSER_SET_TIMEOUT_RX)
		ctx->config.rx_timeout = config->rx_timeout;
	if (config->config_mask & RTSER_SET_TIMEOUT_TX)
		ctx->config.tx_timeout = config->tx_timeout;
	if (config->config_mask & RTSER_SET_TIMEOUT_EVENT)
		ctx->config.event_timeout = config->event_timeout;
    
    if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
		if (config->timestamp_history & RTSER_RX_TIMESTAMP_HISTORY) {
			if (!ctx->in_history) {
				ctx->in_history = *in_history_ptr;
				*in_history_ptr = NULL;
				if (!ctx->in_history)
					err = -ENOMEM;
			}
		} else {
			*in_history_ptr = ctx->in_history;
			ctx->in_history = NULL;
		}
	}

	if (config->config_mask & RTSER_SET_EVENT_MASK) {
		ctx->config.event_mask = config->event_mask & EVENT_MASK;
		ctx->ioc_events = 0;

		if ((config->event_mask & RTSER_EVENT_RXPEND) &&
		    (ctx->in_npend > 0))
			ctx->ioc_events |= RTSER_EVENT_RXPEND;

		if ((config->event_mask & RTSER_EVENT_ERRPEND)
		    && ctx->status)
			ctx->ioc_events |= RTSER_EVENT_ERRPEND;
	}

	if (config->config_mask & RTSER_SET_HANDSHAKE) {
		ctx->config.handshake = config->handshake;

		switch (ctx->config.handshake) {
		case RTSER_RTSCTS_HAND:
			/* ...? */

		default:	/* RTSER_NO_HAND */
			ctx->mcr_status = RTSER_MCR_RTS | RTSER_MCR_OUT1;
			break;
		}
        rt_amba_uart_set_mcr(ctx, ctx->mcr_status);
	}

	/* configure hardware with new parameters */
	if (config->config_mask & (RTSER_SET_BAUD |
				   RTSER_SET_PARITY |
				   RTSER_SET_DATA_BITS |
				   RTSER_SET_STOP_BITS |
				   RTSER_SET_EVENT_MASK |
				   RTSER_SET_HANDSHAKE)) {
		
		unsigned int lcr_h, old_cr;
		unsigned int baud, quot, clkdiv;

		//if (uap->vendor->oversampling)
		//	clkdiv = 8;
		//else
		//	clkdiv = 16;

		/*
		* Ask the core to calculate the divisor for us.
		*/
		baud = ctx->config.baud_rate;
        
		if (baud > uap->uartclk/16)
			quot = DIV_ROUND_CLOSEST(uap->uartclk * 8, baud);
		else
			quot = DIV_ROUND_CLOSEST(uap->uartclk * 4, baud);
		
		rtdm_printk("data_bits=%d\n", ctx->config.data_bits);
		switch (ctx->config.data_bits) {
		case RTSER_5_BITS:
			lcr_h = UART01x_LCRH_WLEN_5;
			break;
		case RTSER_6_BITS:
			lcr_h = UART01x_LCRH_WLEN_6;
			break;
		case RTSER_7_BITS:
			lcr_h = UART01x_LCRH_WLEN_7;
			break;
		case RTSER_8_BITS:
		default: // CS8
			lcr_h = UART01x_LCRH_WLEN_8;
			break;
		}
		
		if (ctx->config.stop_bits == RTSER_2_STOPB)
			lcr_h |= UART01x_LCRH_STP2;
		if (ctx->config.parity == RTSER_ODD_PARITY ||
		    ctx->config.parity == RTSER_EVEN_PARITY) {
			
			lcr_h |= UART01x_LCRH_PEN;
			if (ctx->config.parity == RTSER_ODD_PARITY)
				lcr_h &= ~UART01x_LCRH_EPS;
			else if (ctx->config.parity == RTSER_EVEN_PARITY)
				lcr_h |= UART01x_LCRH_EPS;
		}
		
        rtdm_printk("fifosize = %d", uap->fifosize);
		if (uap->fifosize > 1)
			lcr_h |= UART01x_LCRH_FEN;//enable fifo

		//pl011_setup_status_masks(port, termios); 
		
		if (config->event_mask &
		    (RTSER_EVENT_MODEMHI | RTSER_EVENT_MODEMLO))
			rt_pl011_enable_ms(ctx);

		/* first, disable everything */
		old_cr = pl011_read(uap, REG_CR);
		pl011_write(0, uap, REG_CR);
		
		if (uap->have_rtscts) {
			if (old_cr & UART011_CR_RTS)
				old_cr |= UART011_CR_RTSEN;

			old_cr |= UART011_CR_CTSEN;
			ctx->status |= UPSTAT_AUTOCTS | UPSTAT_AUTORTS;
		} else {
			old_cr &= ~(UART011_CR_CTSEN | UART011_CR_RTSEN);
			ctx->status &= ~(UPSTAT_AUTOCTS | UPSTAT_AUTORTS);
		}

		if (uap->vendor->oversampling) {
			if (baud > uap->uartclk / 16)
				old_cr |= ST_UART011_CR_OVSFACT;
			else
				old_cr &= ~ST_UART011_CR_OVSFACT;
		}

		/*
		 * Workaround for the ST Micro oversampling variants to
		 * increase the bitrate slightly, by lowering the divisor,
		 * to avoid delayed sampling of start bit at high speeds,
		 * else we see data corruption.
		 */
		if (uap->vendor->oversampling) {
			if ((baud >= 3000000) && (baud < 3250000) && (quot > 1))
				quot -= 1;
			else if ((baud > 3250000) && (quot > 2))
				quot -= 2;
		}
		/* Set baud rate */
		pl011_write(quot & 0x3f, uap, REG_FBRD);
		pl011_write(quot >> 6, uap, REG_IBRD);

		/*
		 * ----------v----------v----------v----------v-----
		 * NOTE: REG_LCRH_TX and REG_LCRH_RX MUST BE WRITTEN AFTER
		 * REG_FBRD & REG_IBRD.
		 * ----------^----------^----------^----------^-----
		 */
		pl011_write_lcr_h(uap, lcr_h);
		pl011_write(old_cr, uap, REG_CR);

		//rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
    }
    
	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
	return err;
}

void rt_amba_uart_cleanup_ctx(struct rt_amba_uart_ctx *ctx)
{
	rtdm_event_destroy(&ctx->in_event);
	rtdm_event_destroy(&ctx->out_event);
	rtdm_event_destroy(&ctx->ioc_event);
	rtdm_mutex_destroy(&ctx->out_lock);
}

static int rt_pl011_uart_open(struct rtdm_fd *fd, int oflags)
{
    struct rt_amba_uart_port *uap=NULL;
	struct rt_amba_uart_ctx *ctx=NULL;
    unsigned int cr;
    int retval;
    
    rtdm_printk("%s", __FUNCTION__);
    
    // rtdm init
    ctx = rtdm_fd_to_private(fd);
	ctx->port = (struct rt_amba_uart_port *)rtdm_fd_device(fd)->device_data;
    
	uap = ctx->port;

	/* IPC initialisation - cannot fail with used parameters */
	rtdm_lock_init(&ctx->lock);
	rtdm_event_init(&ctx->in_event, 0);
	rtdm_event_init(&ctx->out_event, 0);
	rtdm_event_init(&ctx->ioc_event, 0);
	rtdm_mutex_init(&ctx->out_lock);
    
    ctx->in_head = 0;
	ctx->in_tail = 0;
	ctx->in_npend = 0;
	ctx->in_nwait = 0;
	ctx->in_lock = 0;
	ctx->in_history = NULL;

	ctx->out_head = 0;
	ctx->out_tail = 0;
	ctx->out_npend = 0;

	ctx->ioc_events = 0;
	ctx->ioc_event_lock = 0;
	ctx->status = 0;
	ctx->saved_errors = 0;
    
    
    // hwinit
    retval = pl011_hwinit(uap);
	if (retval)
		goto clk_dis;
    
    /* set im when after reset hw and before enable irq */
    pl011_write(uap->im, uap, REG_IMSC);
    
    pl011_write(uap->vendor->ifls, uap, REG_IFLS);
    
    rtdm_lock_get(&ctx->lock);
    
    /* restore RTS and DTR */
	cr = uap->old_cr & (UART011_CR_RTS | UART011_CR_DTR);
	cr |= UART01x_CR_UARTEN | UART011_CR_RXE | UART011_CR_TXE;
	pl011_write(cr, uap, REG_CR);
    
    rtdm_lock_put(&ctx->lock);
    
    /* initialise the old status of the modem signals */
    uap->old_status = pl011_read(uap, REG_FR) & UART01x_FR_MODEM_ANY;
    
    // irq
    retval = rtdm_irq_request(&ctx->irq_handle,
				uap->irq, rt_pl011_int, 0,
				rtdm_fd_device(fd)->name, ctx);
    
    pl011_enable_interrupts(ctx);
    return retval;
    
clk_dis:
    return retval;
}

void rt_pl011_uart_close(struct rtdm_fd *fd)
{
    struct rt_amba_uart_port *uap=NULL;
	struct rt_amba_uart_ctx *ctx=NULL;
	rtdm_printk("%s", __FUNCTION__);
	
	ctx = rtdm_fd_to_private(fd);
	uap = ctx->port;
    
    pl011_disable_interrupts(ctx);
    rtdm_irq_free(&ctx->irq_handle);
    
    pl011_disable_uart(ctx);
    
    rt_amba_uart_cleanup_ctx(ctx);
	kfree(ctx->in_history);
    
    /* Shut down the clock producer */
	clk_disable_unprepare(uap->clk);
    
    /* Optionally let pins go into sleep states */
	pinctrl_pm_select_sleep_state(uap->dev);

	if (dev_get_platdata(uap->dev)) {
		struct amba_pl011_data *plat;

		plat = dev_get_platdata(uap->dev);
		if (plat->exit)
			plat->exit();
	}
}


static int rt_pl011_uart_ioctl(struct rtdm_fd *fd,
			     unsigned int request, void *arg)
{
	rtdm_lockctx_t lock_ctx;
    struct rt_amba_uart_port *uap;
	struct rt_amba_uart_ctx *ctx;
	int err = 0;
    printk("%s", __FUNCTION__);
    
	ctx = rtdm_fd_to_private(fd);
    uap = ctx->port;
    
	switch (request) {
	case RTSER_RTIOC_GET_CONFIG:
		if (rtdm_fd_is_user(fd))
			err = rtdm_safe_copy_to_user(fd, arg,
						   &ctx->config,
						   sizeof(struct rtser_config));
		else
			memcpy(arg, &ctx->config,
			       sizeof(struct rtser_config));
		break;

	case RTSER_RTIOC_SET_CONFIG: {
    	struct rtser_config *config;
		struct rtser_config config_buf;
		uint64_t *hist_buf = NULL;

		/*
		 * We may call regular kernel services ahead, ask for
		 * re-entering secondary mode if need be.
		 */
		if (rtdm_in_rt_context())
			return -ENOSYS;

		config = (struct rtser_config *)arg;
		if (rtdm_fd_is_user(fd)) {
			err = rtdm_safe_copy_from_user(fd, &config_buf,
						     arg, sizeof(struct rtser_config));
			if (err)
				return err;

			config = &config_buf;
		}
        
        printk("baud_rate=%d, uartclk = %d", config->baud_rate, uap->uartclk);
		if ((config->config_mask & RTSER_SET_BAUD) &&
		    (config->baud_rate > uap->uartclk / 16 ||
		     config->baud_rate <= 0))
			/* invalid baudrate for this port */
			return -EINVAL;

		if (config->config_mask & RTSER_SET_TIMESTAMP_HISTORY) {
			if (config->timestamp_history & RTSER_RX_TIMESTAMP_HISTORY)
				hist_buf = kmalloc(IN_BUFFER_SIZE * sizeof(nanosecs_abs_t), GFP_KERNEL);
		}
        
		rt_amba_uart_set_config(ctx, config, &hist_buf);

		if (hist_buf)
			kfree(hist_buf);
		break;
	}
    
    case RTSER_RTIOC_GET_STATUS: {
        break;
    }
    
    case RTSER_RTIOC_GET_CONTROL:{
        break;
    }
    
    case RTSER_RTIOC_SET_CONTROL: {
        break;
    }
    
    case RTSER_RTIOC_WAIT_EVENT: {
        break;
    }
    
    case RTSER_RTIOC_BREAK_CTL: {
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
		rt_amba_uart_break_ctl(ctx, (unsigned long)arg);
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		break;
	}
    
	default:
		err = -ENOTTY;
	}

	return err;
}

ssize_t rt_pl011_uart_read(struct rtdm_fd *fd, void *buf, size_t nbyte)
{
	struct rt_amba_uart_ctx *ctx;
	rtdm_lockctx_t lock_ctx;
	size_t read = 0;
	int pending;
	int block;
	int subblock;
	int in_pos;
	char *out_pos = (char *)buf;
	rtdm_toseq_t timeout_seq;
	ssize_t ret = -EAGAIN;	/* for non-blocking read */
	int nonblocking;
    
    rtdm_printk("%s", __FUNCTION__);
	if (nbyte == 0)
		return 0;
	
	if (rtdm_fd_is_user(fd) && !rtdm_rw_user_ok(fd, buf, nbyte))
		return -EFAULT;

	ctx = rtdm_fd_to_private(fd);
	rtdm_toseq_init(&timeout_seq, ctx->config.rx_timeout);

	/* non-blocking is handled separately here */
	nonblocking = (ctx->config.rx_timeout < 0);
    
    /* only one reader allowed, stop any further attempts here */
	if (test_and_set_bit(0, &ctx->in_lock))
		return -EBUSY;
    
    rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
    
    while (1) {
        if (ctx->status) {
			if (ctx->status & RTSER_LSR_BREAK_IND)
				ret = -EPIPE;
			else
				ret = -EIO;
			ctx->saved_errors = ctx->status &
			    (RTSER_LSR_OVERRUN_ERR | RTSER_LSR_PARITY_ERR |
			     RTSER_LSR_FRAMING_ERR | RTSER_SOFT_OVERRUN_ERR);
			ctx->status = 0;
			break;
		}
        
        pending = ctx->in_npend;
        
        if (pending > 0) {
			block = subblock = (pending <= nbyte) ? pending : nbyte;
			in_pos = ctx->in_head;
            
			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
            
            /* Do we have to wrap around the buffer end? */
			if (in_pos + subblock > IN_BUFFER_SIZE) {
				/* Treat the block between head and buffer end
				 * separately.
				 */
				subblock = IN_BUFFER_SIZE - in_pos;

				if (rtdm_fd_is_user(fd)) {
					if (rtdm_copy_to_user
					    (fd, out_pos,
					     &ctx->in_buf[in_pos],
					     subblock) != 0) {
						ret = -EFAULT;
						goto break_unlocked;
					}
				} else
					memcpy(out_pos, &ctx->in_buf[in_pos], subblock);

				read += subblock;
				out_pos += subblock;
				subblock = block - subblock;
				in_pos = 0;
			}

			if (rtdm_fd_is_user(fd)) {
				if (rtdm_copy_to_user(fd, out_pos,
						      &ctx->in_buf[in_pos],
						      subblock) != 0) {
					ret = -EFAULT;
					goto break_unlocked;
				}
			} else
				memcpy(out_pos, &ctx->in_buf[in_pos], subblock);

			read += subblock;
			out_pos += subblock;
			nbyte -= block;

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

			ctx->in_head = (ctx->in_head + block) & (IN_BUFFER_SIZE - 1);
			ctx->in_npend -= block;
			if (ctx->in_npend == 0)
				ctx->ioc_events &= ~RTSER_EVENT_RXPEND;

			if (nbyte == 0)
				break; /* All requested bytes read. */

			continue;
        }

		if (nonblocking)
			/* ret was set to EAGAIN in case of a real
			 * non-blocking call or contains the error
			 * returned by rtdm_event_wait[_until]
			 */
			break;

		ctx->in_nwait = nbyte;
		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
		ret = rtdm_event_timedwait(&ctx->in_event,
					   ctx->config.rx_timeout,
					   &timeout_seq);
		if (ret < 0) {
			if (ret == -EIDRM) {
				/* Device has been closed - return immediately */
				return -EBADF;
			}

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
			nonblocking = 1;
			if (ctx->in_npend > 0) {
				/* Final turn: collect pending bytes before exit. */
				continue;
			}

			ctx->in_nwait = 0;
			break;
		}
    
        rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
	}

	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

break_unlocked:
	/* Release the simple reader lock, */
	clear_bit(0, &ctx->in_lock);

	if ((read > 0) && ((ret == 0) || (ret == -EAGAIN) ||
			   (ret == -ETIMEDOUT)))
		ret = read;
    
	return ret;
}

static ssize_t rt_pl011_uart_write(struct rtdm_fd *fd, const void *buf,
				size_t nbyte)
{
	struct rt_amba_uart_ctx *ctx;
	rtdm_lockctx_t lock_ctx;
	size_t written = 0;
	int free;
	int block;
	int subblock;
	int out_pos;
	char *in_pos = (char *)buf;
	rtdm_toseq_t timeout_seq;
	ssize_t ret;
    rtdm_printk("%s\n", __FUNCTION__);

	if (nbyte == 0)
		return 0;

	if (rtdm_fd_is_user(fd) && !rtdm_read_user_ok(fd, buf, nbyte))
		return -EFAULT;

	ctx = rtdm_fd_to_private(fd);

	rtdm_toseq_init(&timeout_seq, ctx->config.rx_timeout);

	/* Make write operation atomic. */
	ret = rtdm_mutex_timedlock(&ctx->out_lock, ctx->config.rx_timeout,
				   &timeout_seq);
	if (ret)
		return ret;

	while (nbyte > 0) {
		rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

		free = OUT_BUFFER_SIZE - ctx->out_npend;

		if (free > 0) {
			block = subblock = (nbyte <= free) ? nbyte : free;
			out_pos = ctx->out_tail;

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

			/* Do we have to wrap around the buffer end? */
			if (out_pos + subblock > OUT_BUFFER_SIZE) {
				/* Treat the block between head and buffer
				 * end separately.
				 */
				subblock = OUT_BUFFER_SIZE - out_pos;

				if (rtdm_fd_is_user(fd)) {
					if (rtdm_copy_from_user
					    (fd,
					     &ctx->out_buf[out_pos],
					     in_pos, subblock) != 0) {
						ret = -EFAULT;
						break;
					}
				} else
					memcpy(&ctx->out_buf[out_pos], in_pos,
					       subblock);

				written += subblock;
				in_pos += subblock;

				subblock = block - subblock;
				out_pos = 0;
			}

			if (rtdm_fd_is_user(fd)) {
				if (rtdm_copy_from_user
				    (fd, &ctx->out_buf[out_pos],
				     in_pos, subblock) != 0) {
					ret = -EFAULT;
					break;
				}
			} else
				memcpy(&ctx->out_buf[out_pos], in_pos, block);

			written += subblock;
			in_pos += subblock;
			nbyte -= block;

			rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

			ctx->out_tail =
			    (ctx->out_tail + block) & (OUT_BUFFER_SIZE - 1);
			ctx->out_npend += block;

			ctx->ier_status |= IER_TX;
			rt_amba_uart_start_tx(ctx);

			rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);
			continue;
		}

		rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

		ret = rtdm_event_timedwait(&ctx->out_event,
					   ctx->config.tx_timeout,
					   &timeout_seq);
		if (ret < 0) {
			if (ret == -EIDRM) {
				/* Device has been closed -
				 * return immediately.
				 */
				ret = -EBADF;
			}
			break;
		}
	}

	rtdm_mutex_unlock(&ctx->out_lock);

	if ((written > 0) && ((ret == 0) || (ret == -EAGAIN) ||
			      (ret == -ETIMEDOUT)))
		ret = written;

	return ret;
}

static struct rtdm_driver pl011_uart_driver = {
	.profile_info		= RTDM_PROFILE_INFO(imx_uart,
						    RTDM_CLASS_SERIAL,
						    RTDM_SUBCLASS_16550A,
						    RTSER_PROFILE_VER),
	.device_count		= RT_PL011_UART_MAX,
	.device_flags		= RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.context_size		= sizeof(struct rt_amba_uart_ctx),
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
	struct rt_amba_uart_port *uap;
	struct vendor_data *vendor = id->data;
	int portnr, ret;
	printk("%s:%d", __FUNCTION__, __LINE__);

	portnr = pl011_find_free_port();
	if (portnr < 0)
		return portnr;

	uap = devm_kzalloc(&dev->dev, sizeof(*uap), GFP_KERNEL);
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
    printk("irq = %d\n", dev->irq[0]);
    
    uap->have_rtscts = 1;
    
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
	
    printk("id=%d, tx_fifo=%d", id->id, tx_fifo[id->id]);
	if (!tx_fifo[id->id] || tx_fifo[id->id] > TX_FIFO_SIZE)
		uap->tx_fifo = TX_FIFO_SIZE;
	else
		uap->tx_fifo = tx_fifo[id->id];
    
	uap->use_hwflow = 1;
	
	ret = rtdm_dev_register(rtdm_dev);
	if (ret)
		return ret;

	amba_set_drvdata(dev, uap);
	
	printk("%s on AMBA UART%d: membase=0x%p irq=%d uartclk=%d\n",
	       rtdm_dev->name, id->id, uap->membase, uap->irq, uap->uartclk);

	return 0;
}

static void rt_pl011_remove(struct amba_device *dev)
{
	struct rt_amba_uart_port *uap = amba_get_drvdata(dev);
	struct rtdm_device *rtdm_dev = &uap->rtdm_dev;
    
    amba_set_drvdata(dev, NULL);
    
    /* Shut down the clock producer */
	//clk_disable_unprepare(uap->clk);
    
	rtdm_dev_unregister(rtdm_dev);
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

    ret = amba_driver_register(&rt_pl011_uart_driver);
	if (ret) {
		pr_err("%s; Could not register  driver (err=%d)\n",
			__func__, ret);
	}

	return ret;
}

static void __exit rt_pl011_uart_exit(void)
{
    amba_driver_unregister(&rt_pl011_uart_driver);
}

module_init(rt_pl011_uart_init);
module_exit(rt_pl011_uart_exit);
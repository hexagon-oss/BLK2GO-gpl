// SPDX-License-Identifier: GPL-2.0
/*
 * DMA module for IMX sensors connected to MPF FPGA.
 * There is one set of shared registers and a register set for each IMX channel
 * (imx_ctrl). Every minor device can change the shared registers via IOCTLs.
 */

/* define before device.h to get dev_dbg output */
/* #define DEBUG */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>	/* dma_set_mask */
#include <linux/interrupt.h>
#include <linux/io.h>		/* ioremap */
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>	/* copy_to_user */

/* Generated header file (see FPGA repository) */
#include "iod_delay_control.h"

/* ioctl and others; shared with user space */
#include "mpf_imx_dev.h"

#ifdef MPF_TRACE_REGS
#define iowrite32(val, addr) _iowrite32(val, addr)
#define ioread32(addr) _ioread32(addr)

static inline void _iowrite32(u32 value, volatile void __iomem *addr)
{
	pr_info("mpf_imx(w): %p <= %08x\n", addr, value);
	writel(value, addr);
}

static inline u32 _ioread32(const volatile void __iomem *addr)
{
	u32 value;

	value = readl(addr);
	pr_info("mpf_imx(r): %p => %08x\n", addr, value);

	return value;
}
#endif

#define MS_TO_MPF_TICKS(ms) ((ms) * 1000 * 125) /* 125 MHz ticks, 8 ns */
#define MPF_TICKS_TO_MS(ticks) ((ticks) * 8 / (1000 * 1000))

#define IMX_DMA_TIMEOUT_MS	2000	/* in ms */
#define IMX_DMA_TIMEOUT_VAL	MS_TO_MPF_TICKS(IMX_DMA_TIMEOUT_MS)

#define IMX_EXPOSURE_DEFAULT	8	/* in ms */
#define IMX_EXPOSURE_VAL_0	MS_TO_MPF_TICKS(IMX_EXPOSURE_DEFAULT)
#define IMX_EXPOSURE_VAL_1	MS_TO_MPF_TICKS(IMX_EXPOSURE_DEFAULT)
#define IMX_EXPOSURE_VAL_2	MS_TO_MPF_TICKS(IMX_EXPOSURE_DEFAULT)

#define IMX_XTRIG_DELAY_MS	10	/* in ms */
#define IMX_XTRIG_DELAY_VAL	MS_TO_MPF_TICKS(IMX_XTRIG_DELAY_MS)

#define DRIVER_NAME "mpf_imx"
#define CLASS_NAME  "mpf_imx"

/* PCI slave window size AXI4 interconnect, see FPGA address layout */
#define MPF_PCI_SLAVE_WIN_SZ	SZ_16M
/* DMA window size, subwindows ATR0 to ATR3 */
#define MPF_DMA_WIN_SZ	SZ_4M
/* PCIE0 slave window 0, used as base address. This driver uses window 3-5. */
#define MPF_ATR_WIN_BASE_NUM_IMX	3
#define ATR0_AXI4_SLV0_SRCADDR_PARAM	0x0800
#define ATR0_AXI4_SLV0_SRC_ADDR		0x0804
#define ATR0_AXI4_SLV0_TRSL_ADDR_LSB	0x0808
#define ATR0_AXI4_SLV0_TRSL_ADDR_UDW	0x080c
#define ATR0_AXI4_SLV0_TRSL_PARAM	0x0810

/* list of DMA buffers */
struct fb_allocs {
	struct list_head list;
	struct mpf_imx_buf imx_buf;
};

/* Shared for all sensors */
struct imx_shared_regs {
	uint32_t main_ctrl;
#define MAIN_CTRL_RESET		BIT(0)
#define MAIN_CTRL_START		BIT(1)
#define MAIN_CTRL_CANCEL	BIT(2)
	uint32_t xtrig_delay;	/* in 125 MHz clock cycles */
	uint32_t ts_lsb;	/* timestamp lower 32 bit */
	uint32_t ts_msb;	/* timestamp upper 32 bit */
	uint32_t rx_timeout;	/* in 125 MHz clock cycles, default 1 s */
};

/* Per sensor regs */
struct imx_ctrl {
	uint32_t config;
#define CONFIG_EN_SENSOR	BIT(0)
#define CONFIG_EN_IRQ		BIT(1)
#define CONFIG_CLR_IRQ		BIT(2)
	uint32_t status;
#define STATUS_RESET_DONE	BIT(0)
#define STATUS_IN_PROGRESS	BIT(1)
#define STATUS_SENSOR_BUSY	BIT(2)
#define STATUS_IRQ_PENDING	BIT(3)
	uint32_t exposure;	/* in 125 MHz clock cycles */
	uint32_t dma_addr;	/* size is fixed */
	uint32_t info_tag;
};

#define MPF_IMX_N_SENSORS	3

/* driver data */
struct mpf_imx {
	struct platform_device *pdev;
	struct device *dev;
	struct cdev cdev;
	struct regmap *regmap;
	struct imx_ctrl *imx_ctrl[MPF_IMX_N_SENSORS];
	struct imx_shared_regs *shared_regs;
	void __iomem *iod_base;
	struct regulator *regulator;
	unsigned long sensor_open_mask;		/* modified in open/release */
	unsigned long sensor_ready_mask;	/* modified in start/stop */
	struct task_struct *start_thread;
	ktime_t start_timestamp;
	wait_queue_head_t start_waitq;
	uint32_t xtrig_delay_ms;
	uint32_t exposure_ms[MPF_IMX_N_SENSORS];
};

/* per sensor data */
static struct mpf_imx_sensor {
	const char *name;
	int minor;
	struct device *dev;
	struct imx_ctrl *imx_ctrl;
	struct mpf_imx *mpf_imx;
	spinlock_t lock;
	unsigned int dma_done;
	unsigned int dma_canceled;
	struct mpf_imx_capinfo capinfo;	/* perf and error info */
	unsigned int n_sample_bufs;
	struct fb_allocs sample_buf[MPF_IMX_MAX_BUFS];
	struct dma_pool *dma_pool;
	size_t dma_chunk_size;		/* rounded up frame size */
	struct list_head ready_q;
	struct list_head done_q;
	struct list_head working_q;
	wait_queue_head_t sample_buf_waitq;
	uint32_t sequence;		/* sequence number for sample buf */

	atomic_t		started;
	struct work_struct	irq_work;
	#define IMX_WORK_TO_IMX_SENSOR(w) container_of(w,	\
					    struct mpf_imx_sensor,	\
					    irq_work)
	int		irq;
	int exposure;	/* exposure preset, in 125 MHz ticks */
} devlist[] = {
	[0] = {
		.name = "mpf_imx-0",
		.n_sample_bufs = MPF_IMX_N_BUFS,
		.dma_chunk_size = MPF_IMX_FRAME_SIZE,
		.exposure = IMX_EXPOSURE_VAL_0,
	},
	[1] = {
		.name = "mpf_imx-1",
		.n_sample_bufs = MPF_IMX_N_BUFS,
		.dma_chunk_size = MPF_IMX_FRAME_SIZE,
		.exposure = IMX_EXPOSURE_VAL_1,
	},
	[2] = {
		.name = "mpf_imx-2",
		.n_sample_bufs = MPF_IMX_N_BUFS,
		.dma_chunk_size = MPF_IMX_FRAME_SIZE,
		.exposure = IMX_EXPOSURE_VAL_2,
	},
};

static struct class *mpf_class;	/* from parent */
static dev_t mpf_devt;

/* for debug purposes */
static unsigned int mpf_imx_irq_count;

/* The parent driver setup a 2 GiB 1:1 window (upper 2 GiB) */
static int large_pci_win;	/* transitional, will go away */

static void mpf_imx_irq_work(struct work_struct *work);
static irqreturn_t mpf_imx_isr(int irq, void *cookie);

/*
 * find matching allocation/mapping info for given dma_addr
 */
static struct mpf_imx_buf *find_alloc_info(struct mpf_imx_sensor *sensor,
							uint32_t dma_addr)
{
	int i;

	for (i = 0; i < sensor->n_sample_bufs; i++) {
		if (sensor->sample_buf[i].imx_buf.offset == dma_addr)
			return &sensor->sample_buf[i].imx_buf;
	}

	return NULL;
}

static void mpf_imx_free_bufs(struct mpf_imx_sensor *sensor)
{
	int i;
	dma_addr_t dma_addr;
	void *vaddr;
	size_t size;
	struct device *dev = sensor->dev;
	struct dma_pool *pool = sensor->dma_pool;

	for (i = 0; i < sensor->n_sample_bufs; i++) {
		vaddr = sensor->sample_buf[i].imx_buf.vaddr;
		if (vaddr == NULL)
			continue;

		dma_addr = sensor->sample_buf[i].imx_buf.offset;
		size = sensor->sample_buf[i].imx_buf.size;
		dev_dbg(dev, "%s: freeing: size %zd, vaddr %p, dma_addr %p\n",
			__func__, size, vaddr, (void *)dma_addr);
		if (pool)
			dma_pool_free(pool, vaddr, dma_addr);
		else
			dma_free_coherent(dev, size, vaddr, dma_addr);

		sensor->sample_buf[i].imx_buf.vaddr = 0;
	}

	INIT_LIST_HEAD(&sensor->ready_q);
	INIT_LIST_HEAD(&sensor->working_q);
	INIT_LIST_HEAD(&sensor->done_q);
}

static void disable_dma_win(struct mpf_imx_sensor *sensor)
{
	unsigned int regoffs;
	struct mpf_imx *mpf_imx = sensor->mpf_imx;

	/* ATR 0-2 are used by mpf_edm, we use 3-5 */
	regoffs = (sensor->minor + MPF_ATR_WIN_BASE_NUM_IMX) * 0x20;
	regmap_write(mpf_imx->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			0);
}

static void setup_dma_win(struct mpf_imx_sensor *sensor, uint32_t pci_dma_addr)
{
	struct mpf_imx *mpf_imx = sensor->mpf_imx;
	int log2;
	uint32_t reg;
	unsigned int regoffs;
	uint32_t axi_dma_addr;

	/* add base offset, ATR0 - ATR2 are used by mpf_edm */
	regoffs = (sensor->minor + MPF_ATR_WIN_BASE_NUM_IMX) * 0x20;

	/* get PCI window part, e.g. f740.0000  & ~003f.ffff-> f740.0000 */
	reg = pci_dma_addr & ~(MPF_DMA_WIN_SZ - 1);
	regmap_write(mpf_imx->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_LSB + regoffs,
			reg);
	regmap_write(mpf_imx->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_UDW + regoffs,
			0);
	/*
	 * ATR0_AXI4_SLV0_SRCADDR_PARAM:
	 * 0 ATR_IMPL: If 1, it indicates that the Translation Address Table is
	 * implemented.
	 * 6:1 ATR_SIZE: Defines the Address Translation Space Size. This space
	 * size in bytes is equal to 2^(ATR_SIZE +1).
	 * Allowed values for this field are from 6'd11 (2^12 = 4 KBytes) to
	 * 6'd63 (2^64 = 18 Extra Bytes) only
	 */
	log2 = fls(MPF_DMA_WIN_SZ) - 1;	/* find last bit set */
	/* get AXI window part, e.g. f740.0000  & 00ff.ffff-> 0040.0000 */
	axi_dma_addr = pci_dma_addr & (MPF_PCI_SLAVE_WIN_SZ - 1);
	/* select DMA window, each minor has a MPF_DMA_WIN_SZ boundary */
	axi_dma_addr &= ~(MPF_DMA_WIN_SZ - 1);	/* 0040.0000 -> 0040.0000 */
	reg = axi_dma_addr;
	reg |= ((log2  - 1) << 1);	/* e.g. 0x2f for a 16 MB window */
	reg |= 1;			/* enable window */
	regmap_write(mpf_imx->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			reg);
}

/*
 * Check if AXI to PCI translation was setup in mpf.ko.
 */
static int is_pci_win_setup(struct mpf_imx *mpf_imx)
{
	uint32_t reg;

	regmap_read(mpf_imx->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM, &reg);
	/* check if 2 GB, enabled, AXI src addr 0x8000.00000 */
	if (reg != 0x8000003d) {
		dev_warn(mpf_imx->dev, "AXI to PCI not setup by mpf.ko\n");
		return 0;
	}
	return 1;
}


static int mpf_imx_alloc_bufs(struct mpf_imx_sensor *sensor)
{
	int rc = 0;
	int i;
	dma_addr_t dma_addr;
	dma_addr_t next_boundary;
	void *vaddr;
	struct dma_pool *pool = sensor->dma_pool;
	size_t size = sensor->dma_chunk_size;

	for (i = 0; i < sensor->n_sample_bufs; i++) {
		if (pool) {
			vaddr = dma_pool_alloc(pool, GFP_KERNEL, &dma_addr);
		} else {
			vaddr = dma_alloc_coherent(sensor->dev, size, &dma_addr,
					GFP_KERNEL);
		}
		if (vaddr == NULL) {
			dev_err(sensor->dev,
				"%s: error allocating DMA buf (size=%zd)\n",
					__func__, size);
			rc = -ENOMEM;
			goto err;
		}
		/* check for boundary crossing */
		next_boundary = (dma_addr & ~(MPF_DMA_WIN_SZ - 1))
							+ MPF_DMA_WIN_SZ;
		if (dma_addr + size > next_boundary) {
			dev_warn(sensor->dev,
				"boundary crossing %pad (0x%zx), next %pad\n",
				&dma_addr, size, &next_boundary);
		}

		*(unsigned int *)vaddr = 0x12345670 + i; /* test pattern */
		dev_dbg(sensor->dev, "dma mapped data %pad is at %p (%zd)\n",
			&dma_addr, vaddr, size);

		sensor->sample_buf[i].imx_buf.offset = (off_t) dma_addr;
		sensor->sample_buf[i].imx_buf.vaddr = vaddr;
		sensor->sample_buf[i].imx_buf.size = size;
		sensor->sample_buf[i].imx_buf.index = i;
	}

err:
	return rc;
}

#ifdef DMA_DEBUG
static void print_slave_atr(struct mpf_imx_sensor *sensor, unsigned int regnum)
{
	unsigned int regoffs;
	uint32_t reg;
	struct mpf_imx *mpf_imx = sensor->mpf_imx;
	struct device *dev = sensor->dev;

	regoffs = regnum * 0x20;

	dev_dbg(dev, "Slave ATR# %u\n", regnum);

	regmap_read(mpf_imx->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_SRCADDR_PARAM: %08x\n", regnum, reg);

	regmap_read(mpf_imx->regmap, ATR0_AXI4_SLV0_SRC_ADDR + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_SRC_ADDR     : %08x\n", regnum, reg);

	regmap_read(mpf_imx->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_LSB + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_TRSL_ADDR_LSB: %08x\n", regnum, reg);

	regmap_read(mpf_imx->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_UDW + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_TRSL_ADDR_UDW: %08x\n", regnum, reg);

	regmap_read(mpf_imx->regmap, ATR0_AXI4_SLV0_TRSL_PARAM + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_TRSL_PARAM   : %08x\n", regnum, reg);
}

static void print_dma_info(struct mpf_imx_sensor *imx)
{
	struct imx_ctrl *regs;
	struct imx_shared_regs *shared_regs;
	unsigned long long to;
	struct mpf_imx *mpf_imx = imx->mpf_imx;
	struct device *dev = mpf_imx->dev;

	regs = imx->imx_ctrl;
	shared_regs = mpf_imx->shared_regs;

	to = ioread32(&shared_regs->rx_timeout) * 8ULL; /* 125 MHz = 8 ns */
	to /= 1000000ULL;	/* ms */
	dev_dbg(dev, "timeout: 0x%x (%llu ms)\n",
			ioread32(&shared_regs->rx_timeout), to);
	dev_dbg(dev, "dma_addr: 0x%08x\n", ioread32(&regs->dma_addr));
	/*
	 * Take translation window into account
	 * ATR0_AXI4_SLV0_SRCADDR_PARAM
	 */
	print_slave_atr(imx, 0);
	print_slave_atr(imx, 1);
	print_slave_atr(imx, 2);
}
#endif

/*
 * Stop sensor. We must wait until DMA is done; active cancellation does not
 * work. Even then the DMA continues with dummy data (FPGA limitation).
 */
static void mpf_imx_stop(struct mpf_imx_sensor *imx)
{
	struct mpf_imx *mpf_imx;
	struct imx_ctrl *regs;
	uint32_t status_reg;
	uint32_t ctrl_reg;
	uint32_t val;
	unsigned long timeout;

	/* Do nothing if already stopped. */
	if (!atomic_read(&imx->started)) {
		dev_dbg(imx->dev, "%s: already stopped\n", __func__);
		return;
	}

	atomic_set(&imx->started, 0);

	regs = imx->imx_ctrl;
	val = ioread32(&regs->config);
	dev_dbg(imx->dev, "%s: config reg 0x%08x\n", __func__, val);
	val = ioread32(&regs->status);
	dev_dbg(imx->dev, "%s: status reg 0x%08x\n", __func__, val);

	/* clear pending IRQ */
	ctrl_reg = ioread32(&regs->config);
	ctrl_reg |= CONFIG_CLR_IRQ;
	iowrite32(ctrl_reg, &regs->config);

	val = ioread32(&regs->status);
	dev_dbg(imx->dev, "%s: status reg 0x%08x\n", __func__, val);

	status_reg = ioread32(&regs->status);
	if (status_reg & STATUS_IN_PROGRESS) {
		dev_dbg(imx->dev, "%s: DMA still active, status_reg %08x\n",
				__func__, status_reg);
		/*
		 * NOTE: we can only cancel the transfer for all IMX
		 * altogether, FPGA limitation.
		 */
#ifdef CANCEL_BUG_FIXED
		/*
		 * XXX: FPGA bug, cancel gets sticky if actively set, FPGA
		 * reset is then needed.
		 */
		mpf_imx = imx->mpf_imx;
		iowrite32(MAIN_CTRL_CANCEL, &mpf_imx->shared_regs->main_ctrl);
		/*
		 * Make sure the abort reached the target and is sync'ed.
		 */
		ctrl_reg = ioread32(&mpf_imx->shared_regs->main_ctrl);
#endif
		timeout = jiffies + HZ;
		while (ioread32(&regs->status) & STATUS_IN_PROGRESS) {
			if (time_after(jiffies, timeout)) {
				/*
				 * This can be a serious problem if the DMA is
				 * still transferring to a buffer which will be
				 * freed and used by the system otherwise.
				 */
				dev_err(imx->dev,
					"%s: DMA still active after abort\n",
					__func__);
				break;
			}
			msleep(20);
		}
	}

	cancel_work_sync(&imx->irq_work);

	/* disable sensor */
	iowrite32(0, &regs->config);

	mpf_imx = imx->mpf_imx;
	clear_bit(imx->minor, &mpf_imx->sensor_ready_mask);
}

/*
 * Wait until sensors are ready, there is no IRQ for it.
 */
static int mpf_imx_ready_wait(struct mpf_imx *mpf_imx)
{
	int i;
	unsigned long timeout;
	struct imx_ctrl *regs;

	timeout = jiffies + HZ;
	for (i = 0; i < MPF_IMX_N_SENSORS; i++) {
		regs = mpf_imx->imx_ctrl[i];
		while (ioread32(&regs->status) & STATUS_SENSOR_BUSY) {
			if (time_after(jiffies, timeout)) {
				dev_err(mpf_imx->dev,
			"%s: sensor %d is busy, cannot start new transfer.\n",
				__func__, i);
				return -EBUSY;
			}
		}
	}

	return 0;
}

/*
 * Start DMA from FGPA to onboard memory.
 * DMA address must be acquired via IMX_IOC_ALLOCBUFS.
 */
static ssize_t mpf_imx_start(struct mpf_imx_sensor *sensor, uint32_t dma_addr)
{
	//int rc;
	struct mpf_imx_buf *imx_buf;
	struct mpf_imx *mpf_imx;
	struct imx_ctrl *regs;
	struct device *dev = sensor->dev;

	/* XXX: check if DMA is already running */

	/*
	 * XXX: does not harm, but is not necessary since we are called only
	 * with the DMA addr of a sample_buf
	 */
	/* check if DMA addr is valid */
	imx_buf = find_alloc_info(sensor, dma_addr);
	if (imx_buf == NULL) {
		dev_warn(dev, "%s: invalid DMA addr 0x%08x\n", __func__,
				dma_addr);
		return -EINVAL;
	}

	dev_dbg(dev, "%s: 0x%08x\n", __func__, dma_addr);
	/*
	 * Our CPU DMA address is 32 wide, e.g. 0xf68f0000
	 * => AXI dma_addr := 0x008f.0000 and
	 * ATRx_AXI4_SLV0_TRSL_ADDR_LSB := 0xf680.0000
	 */
	regs = sensor->imx_ctrl;

	if (large_pci_win)
		iowrite32(dma_addr, &regs->dma_addr);
	else {
		/* transitional, will be deleted */
		iowrite32(dma_addr & (MPF_PCI_SLAVE_WIN_SZ - 1),
				&regs->dma_addr);
		setup_dma_win(sensor, dma_addr);
	}

	/*
	 * The DMA len is hard-coded in the FPGA, therefore no dma_len setting
	 * here.
	 */

#ifdef DMA_DEBUG
	print_dma_info(sensor);
#endif

	spin_lock_irq(&sensor->lock);
	/* enable data reception and IRQ */
	iowrite32(CONFIG_EN_SENSOR, &regs->config);
	iowrite32(CONFIG_EN_SENSOR | CONFIG_EN_IRQ, &regs->config);
	spin_unlock_irq(&sensor->lock);

	/* NOTE: we can only start all channels together, FPGA limitation */
	mpf_imx = sensor->mpf_imx;
	set_bit(sensor->minor, &mpf_imx->sensor_ready_mask);
	wake_up_interruptible(&mpf_imx->start_waitq);

	return 0;
}


static int mpf_imx_mmap(struct file *filep, struct vm_area_struct *vma)
{
	unsigned long offset;
	int rc;
	struct mpf_imx_sensor *sensor = filep->private_data;
	struct device *dev = sensor->dev;
	struct mpf_imx_buf *imx_buf;

	dev_dbg(dev,
	"mmap: vm_start: 0x%8.8lx, vm_end: 0x%8.8lx, vm_pgoff: 0x%8.8lx\n",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);

	offset = vma->vm_pgoff << PAGE_SHIFT;

	/* check if it is a DMA mapping */
	imx_buf = find_alloc_info(sensor, offset);
	if (imx_buf == NULL)
		return -EINVAL;

	/* Bufferable, normal memory / non-cacheable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	rc = io_remap_pfn_range(vma, vma->vm_start,
			offset >> PAGE_SHIFT,
			vma->vm_end - vma->vm_start,
			vma->vm_page_prot);
	if (rc) {
		dev_err(dev, "%s: io_remap_pfn_range() error: rc = %d\n",
				__func__, rc);
		return rc;
	}

	return 0;
}

static unsigned int mpf_imx_poll(struct file *filep, poll_table *wait)
{
	struct mpf_imx_sensor *sensor = filep->private_data;
	struct device *dev = sensor->dev;
	unsigned int mask = 0;
	unsigned int dma_done;
	unsigned int dma_canceled;

	poll_wait(filep, &sensor->sample_buf_waitq, wait);

	dev_dbg(dev, "%s mpf_imx_irq_count %u\n", __func__, mpf_imx_irq_count);

	spin_lock_irq(&sensor->lock);
	dma_done = sensor->dma_done;
	dma_canceled = sensor->dma_canceled;

	if (dma_done) {
		sensor->dma_done = 0;
		if (!list_empty(&sensor->done_q)) {
			mask = POLLIN | POLLRDNORM;
			dev_dbg(dev, "%s: data ready\n", __func__);
		} else {
			/* perhaps poll was called after dqbuf? */
			dev_info(dev, "%s: done but no data\n", __func__);
		}
	}
	spin_unlock_irq(&sensor->lock);

	/*
	 * DMA was done, but canceled and filled with dummy data.
	 */
	if (sensor->dma_canceled) {
		mask = POLLERR;
		dev_dbg(dev, "%s canceled\n", __func__);
	}

	return mask;
}


/*
 * start sample buffer streaming
 * This ioctl starts all enabled sensors.
 * It is a HW limitation that we must start all sensors together.
 * But there is a single completion interrupt for each channel.
 */
static int ioctl_start(struct mpf_imx_sensor *sensor, void __user *udata)
{
	int rc;
	struct device *dev = sensor->dev;
	struct mpf_imx *mpf_imx = sensor->mpf_imx;
	uint32_t write_addr;
	struct fb_allocs *bufs;
	unsigned int sensor_mask;
	//struct imx_ctrl *regs = sensor->imx_ctrl;

	/*
	 * XXX: return EBUSY if a capture was already started.
	 */

	dev_dbg(dev, "%s\n", __func__);

	sensor_mask = mpf_imx->sensor_open_mask;
	while (sensor_mask) {
		unsigned int sensor_num;

		sensor_num = __ffs(sensor_mask);
		sensor = &devlist[sensor_num];

		if (list_empty(&sensor->ready_q)) {
			dev_err(dev, "no buffer queued for sensor %u\n",
					sensor_num);
			return -EINVAL;
		}

		bufs = list_entry(sensor->ready_q.next, struct fb_allocs, list);
		dev_dbg(dev, "%s: index %u, DMA addr 0x%08zx\n", __func__,
				bufs->imx_buf.index, bufs->imx_buf.offset);

		/* move buffer from ready_q to working_q */
		list_del(sensor->ready_q.next);
		list_add_tail(&bufs->list, &sensor->working_q);
		write_addr = (ulong) bufs->imx_buf.offset & 0xffffffff;

		spin_lock_irq(&sensor->lock);
		sensor->dma_done = 0;
		sensor->dma_canceled = 0;
		spin_unlock_irq(&sensor->lock);

		/* start DMA */
		rc = mpf_imx_start(sensor, write_addr);
		if (rc)
			return rc;

		bufs->imx_buf.ts = ktime_get_ns();
		dev_dbg(dev, "DMA started, ts %lld\n", bufs->imx_buf.ts);

		atomic_set(&sensor->started, 1);

		sensor_mask &= ~BIT(sensor_num);
	}
	return 0;
}

/*
 * stop sample buffer streaming
 */
static int ioctl_stop(struct mpf_imx_sensor *sensor, void __user *udata)
{
	struct device *dev = sensor->dev;

	dev_dbg(dev, "%s\n", __func__);

	mpf_imx_stop(sensor);

	return 0;
}

static int ioctl_querybuf(struct mpf_imx_sensor *sensor, void __user *udata)
{
	unsigned int index;
	struct mpf_imx_buf imx_buf;
	struct fb_allocs *fb_allocs;
	struct device *dev = sensor->dev;

	if (copy_from_user(&imx_buf, udata, sizeof(imx_buf)))
		return -EFAULT;

	index = imx_buf.index;
	if (index >= sensor->n_sample_bufs) {
		dev_err(dev, "%s: invalid index %u\n", __func__, index);
		return -EINVAL;
	}

	fb_allocs = &sensor->sample_buf[index];
	imx_buf.offset = fb_allocs->imx_buf.offset;
	imx_buf.vaddr = fb_allocs->imx_buf.vaddr;
	imx_buf.size = fb_allocs->imx_buf.size;
	dev_dbg(dev, "%s: index %u. offset %zx, vaddr %p\n", __func__, index,
			imx_buf.offset, imx_buf.vaddr);

	return copy_to_user(udata, &imx_buf, sizeof(imx_buf));
}

static int ioctl_allocbufs(struct mpf_imx_sensor *sensor, void __user *udata)
{
	int i;
	int index;
	int ret;
	struct mpf_imx_buf imx_buf;
	size_t size;
	dma_addr_t dma_addr;
	void *vaddr;

	if (copy_from_user(&imx_buf, udata, sizeof(imx_buf)))
		return -EFAULT;

	size = sensor->dma_chunk_size;

	mpf_imx_free_bufs(sensor);
	ret = mpf_imx_alloc_bufs(sensor);
	if (ret)
		return ret;

	for (i = 0; i < sensor->n_sample_bufs; i++) {
		dma_addr = sensor->sample_buf[i].imx_buf.offset;
		vaddr = sensor->sample_buf[i].imx_buf.vaddr;
		size = sensor->sample_buf[i].imx_buf.size;
		index = sensor->sample_buf[i].imx_buf.index;

		imx_buf.offset = (off_t) dma_addr;
		imx_buf.vaddr = vaddr;
		imx_buf.size = size;
		imx_buf.index = index;
	}

	/*
	 * Put buffers on ready_q.
	 */
	for (i = 0; i < sensor->n_sample_bufs; i++)
		list_add_tail(&sensor->sample_buf[i].list, &sensor->ready_q);

	ret = copy_to_user(udata, &imx_buf, sizeof(imx_buf));

	return ret;
}

/*
 * Get statistic and error status
 */
static int ioctl_capinfo(struct mpf_imx_sensor *mpf_imx, void __user *udata)
{
	int ret;
	struct mpf_imx_capinfo capinfo;

	/*
	 * XXX: fill with useful info
	 */

	ret = copy_to_user(udata, &mpf_imx->capinfo, sizeof(capinfo));

	return ret;
}

static int ioctl_qbuf(struct mpf_imx_sensor *sensor, void __user *udata)
{
	int ret = 0;
	unsigned long lock_flags;
	unsigned int index;
	struct mpf_imx_buf imx_buf;
	struct fb_allocs *ready_buf;
	int restart = 0;

	if (copy_from_user(&imx_buf, udata, sizeof(imx_buf)))
		return -EFAULT;

	index = imx_buf.index;

	dev_dbg(sensor->dev, "QBUF %u\n", index);

	spin_lock_irqsave(&sensor->lock, lock_flags);

	list_add_tail(&sensor->sample_buf[index].list, &sensor->ready_q);
	/*
	 * If streaming is on and working_q is empty, i.e. run out of buffers,
	 * we have to restart,
	 */
	if (atomic_read(&sensor->started)) {
		if (list_empty(&sensor->working_q)) {
			/* Warn, since it is a data overflow. */
			dev_warn(sensor->dev,
			"Data Overflow detected around sequence# %u.\n",
					sensor->sequence);
			restart = 1;
			ready_buf = list_entry(sensor->ready_q.next,
						 struct fb_allocs, list);
			list_del(sensor->ready_q.next);
			list_add_tail(&ready_buf->list, &sensor->working_q);
		}
	}

	spin_unlock_irqrestore(&sensor->lock, lock_flags);

	if (restart)
		ret = mpf_imx_start(sensor, ready_buf->imx_buf.offset);

	return ret;
}

static int ioctl_dqbuf(struct mpf_imx_sensor *sensor, void __user *udata)
{
	int ret;
	struct mpf_imx_buf imx_buf;
	struct fb_allocs *done_buf;
	struct device *dev = sensor->dev;

	if (copy_from_user(&imx_buf, udata, sizeof(imx_buf)))
		return -EFAULT;

	spin_lock_irq(&sensor->lock);
	if (list_empty(&sensor->done_q)) {
		dev_dbg(dev, "%s: no buffer in done queue\n", __func__);
		spin_unlock_irq(&sensor->lock);
		return -EAGAIN;
	}
	done_buf = list_entry(sensor->done_q.next, struct fb_allocs, list);
	if (done_buf == NULL) {
		dev_err(dev, "%s: no buffer in done queue\n", __func__);
		spin_unlock_irq(&sensor->lock);
		return -EAGAIN;
	}
	list_del(sensor->done_q.next);
	spin_unlock_irq(&sensor->lock);

	ret = copy_to_user(udata, &done_buf->imx_buf, sizeof(imx_buf));

	dev_dbg(dev,
		"%s: index %u, offset 0x%zx, val 0x%08x\n", __func__,
		done_buf->imx_buf.index, done_buf->imx_buf.offset,
		*(int *)done_buf->imx_buf.vaddr);
	return ret;
}

static int ioctl_s_ctrl(struct mpf_imx_sensor *sensor, void __user *udata)
{
	struct mpf_imx_ctrl ctrl;
	struct mpf_imx *mpf_imx;
	struct imx_ctrl *ctrl_regs = sensor->imx_ctrl;
	int rc = 0;

	if (copy_from_user(&ctrl, udata, sizeof(ctrl)))
		return -EFAULT;

	switch (ctrl.id) {
	case MPF_IMX_CID_TIMEOUT:
		mpf_imx = sensor->mpf_imx;
		iowrite32(MS_TO_MPF_TICKS(ctrl.val),
				&mpf_imx->shared_regs->rx_timeout);
		break;
	case MPF_IMX_CID_XTRIG_DELAY:
		mpf_imx = sensor->mpf_imx;
		mpf_imx->xtrig_delay_ms = ctrl.val;
		iowrite32(MS_TO_MPF_TICKS(ctrl.val),
				&mpf_imx->shared_regs->xtrig_delay);
		break;
	case MPF_IMX_CID_EXPOSURE:
		/*
		 * XXX: make sure sensor is disabled, otherwise the new exposure
		 * value is ignored by the FPGA.
		 */
		iowrite32(MS_TO_MPF_TICKS(ctrl.val), &ctrl_regs->exposure);
		mpf_imx = sensor->mpf_imx;
		mpf_imx->exposure_ms[sensor->minor] = ctrl.val;
		break;
	case MPF_IMX_CID_TAG:
		iowrite32(ctrl.val, &ctrl_regs->info_tag);
		break;
	default:
		rc = -EINVAL;
	}

	return rc;
}

static int ioctl_g_ctrl(struct mpf_imx_sensor *sensor, void __user *udata)
{
	struct imx_ctrl *ctrl_regs = sensor->imx_ctrl;
	struct mpf_imx_ctrl ctrl;

	if (copy_from_user(&ctrl, udata, sizeof(ctrl)))
		return -EFAULT;

	switch (ctrl.id) {
	case MPF_IMX_CID_EXPOSURE:
		ctrl.val = ioread32(&ctrl_regs->exposure);
		ctrl.val = MPF_TICKS_TO_MS(ctrl.val);
		break;
	default:
		return -EINVAL;
	}

	return copy_to_user(udata, &ctrl, sizeof(ctrl));
}

static long mpf_imx_ioctl(struct file *filep, unsigned int ioc,
				unsigned long arg)
{
	int ret;
	void __user *udata = (void __user *) arg;
	struct mpf_imx_sensor *mpf = filep->private_data;

	switch (ioc) {

	case MPF_IMX_IOC_ALLOCBUFS:
		ret = ioctl_allocbufs(mpf, udata);
		break;

	case MPF_IMX_IOC_QUERYBUF:
		ret = ioctl_querybuf(mpf, udata);
		break;

	case MPF_IMX_IOC_START:
		ret = ioctl_start(mpf, udata);
		break;

	case MPF_IMX_IOC_STOP:
		ret = ioctl_stop(mpf, udata);
		break;

	case MPF_IMX_IOC_CAPINFO:
		ret = ioctl_capinfo(mpf, udata);
		break;

	case MPF_IMX_IOC_QBUF:
		ret = ioctl_qbuf(mpf, udata);
		break;

	case MPF_IMX_IOC_DQBUF:
		ret = ioctl_dqbuf(mpf, udata);
		break;

	case MPF_IMX_IOC_S_CTRL:
		ret = ioctl_s_ctrl(mpf, udata);
		break;

	case MPF_IMX_IOC_G_CTRL:
		ret = ioctl_g_ctrl(mpf, udata);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static int mpf_imx_open(struct inode *inode, struct file *file)
{
	struct mpf_imx_sensor *sensor;
	struct mpf_imx *mpf_imx;
	int minor = iminor(inode);

	if (minor > ARRAY_SIZE(devlist))
		return -ENXIO;

	sensor = &devlist[minor];
	file->private_data = sensor;

	INIT_LIST_HEAD(&sensor->ready_q);
	INIT_LIST_HEAD(&sensor->working_q);
	INIT_LIST_HEAD(&sensor->done_q);
	atomic_set(&sensor->started, 0);

	mpf_imx = sensor->mpf_imx;
	set_bit(minor, &mpf_imx->sensor_open_mask);

	return 0;
}

static int mpf_imx_release(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct mpf_imx_sensor *sensor = &devlist[minor];
	struct mpf_imx *mpf_imx = sensor->mpf_imx;
	struct device *dev = sensor->dev;

	mpf_imx_stop(sensor);

	dev_dbg(dev, "%s\n", __func__);

	mpf_imx = sensor->mpf_imx;
	clear_bit(minor, &mpf_imx->sensor_open_mask);

	mpf_imx_free_bufs(sensor);

	file->private_data = NULL;

	dev_dbg(dev, "%s done\n", __func__);
	return 0;
}


static const struct file_operations mpf_fileops = {
	.owner            = THIS_MODULE,
	.poll		  = mpf_imx_poll,
	.unlocked_ioctl   = mpf_imx_ioctl,
	.mmap             = mpf_imx_mmap,
	.open             = mpf_imx_open,
	.release          = mpf_imx_release,
};

static void mpf_imx_request_irq(struct mpf_imx *mpf_imx)
{
	int irq;
	int minor;
	int rc;
	int num_devices = ARRAY_SIZE(devlist);
	struct mpf_imx_sensor *sensor;

	for (minor = 0; minor < num_devices; minor++) {
		sensor = &devlist[minor];

		irq = platform_get_irq(mpf_imx->pdev, minor);
		if (irq < 0) {
			dev_err(mpf_imx->dev,
				"%s: Could not get IRQ resource %d.\n",
						__func__, minor);
			continue;
		}

		rc = devm_request_threaded_irq(mpf_imx->dev, irq, NULL,
						mpf_imx_isr, 0,
						DRIVER_NAME, sensor);
		if (rc) {
			dev_err(mpf_imx->dev,
				"%s: Can't allocate irq %d\n", __func__,
						irq);
			continue;
		}
		sensor->irq = irq;
	}
}

static int mpf_init_chrdev(struct mpf_imx *mpf_imx)
{
	int rc;
	int minor, major;
	struct device *device;
	int num_devices = ARRAY_SIZE(devlist);

	mpf_class = mpf_imx->dev->parent->class;
	if (!mpf_class) {
		mpf_class = class_create(THIS_MODULE, CLASS_NAME);
		if (IS_ERR(mpf_class)) {
			dev_err(mpf_imx->dev,
					"%s: cannot create class\n", __func__);
			return PTR_ERR(mpf_class);
		}
	}

	rc = alloc_chrdev_region(&mpf_devt, 0, /* minor start */
				 num_devices, DRIVER_NAME);
	if (rc) {
		dev_err(mpf_imx->dev, "%s: Failed to obtain major/minors.\n",
						__func__);
		return rc;
	}

	major = MAJOR(mpf_devt);
	minor = MINOR(mpf_devt);

	cdev_init(&mpf_imx->cdev, &mpf_fileops);
	mpf_imx->cdev.owner = mpf_fileops.owner;
	rc = cdev_add(&mpf_imx->cdev, MKDEV(major, minor), num_devices);
	if (rc) {
		dev_err(mpf_imx->dev, "%s: Failed to add cdev. Aborting.\n",
					__func__);
		goto unregister_chrdev;
	}

	for (minor = 0; minor < num_devices; minor++) {
		struct imx_ctrl *ctrl_regs;

		device = device_create(mpf_class,
				       mpf_imx->dev,
				       MKDEV(major, minor),
				       NULL,
				       "%s", devlist[minor].name);
		if (IS_ERR(device)) {
			dev_err(mpf_imx->dev,
			"%s: Failed to create %s-%d device. Aborting.\n",
					 __func__, DRIVER_NAME, minor);
			rc = -ENODEV;
			goto unroll_device_create;
		}
		devlist[minor].dev = device;
		devlist[minor].mpf_imx = mpf_imx;
		devlist[minor].imx_ctrl = mpf_imx->imx_ctrl[minor];

		init_waitqueue_head(&devlist[minor].sample_buf_waitq);
		spin_lock_init(&devlist[minor].lock);

		INIT_WORK(&devlist[minor].irq_work, mpf_imx_irq_work);

		/*
		 * Create DMA coherent memory pool for each device
		 */
		devlist[minor].minor = minor;
		devlist[minor].dma_pool = dmam_pool_create(DRIVER_NAME,
				mpf_imx->dev,
				devlist[minor].dma_chunk_size,
				MPF_DMA_WIN_SZ, /* align */
				MPF_PCI_SLAVE_WIN_SZ); /* boundary */
		if (!devlist[minor].dma_pool) {
			dev_err(mpf_imx->dev,
				"Failed to create DMA pool for %s, size 0x%zx, boundary 0x%x\n",
				devlist[minor].name,
				devlist[minor].dma_chunk_size,
				MPF_DMA_WIN_SZ);
		}

		/*
		 * Disable PCI slave windows we will use until they are setup.
		 * The FPGA might have enabled them by default.
		 * For example if ATR0 is enabled with wrong settings
		 * and the user opens sensor-fa first (ATR1), then the DMA
		 * will overwrite system memory.
		 */
		disable_dma_win(&devlist[minor]);

		/* presets */
		ctrl_regs = devlist[minor].imx_ctrl;
		iowrite32(devlist[minor].exposure, &ctrl_regs->exposure);
		mpf_imx->exposure_ms[minor] = MPF_TICKS_TO_MS(
						devlist[minor].exposure);
	}

	dev_dbg(mpf_imx->dev, "%s: Created %d device files.\n", __func__,
			num_devices);
	return 0;

unroll_device_create:
	minor--;
	for (; minor >= 0; minor--)
		device_destroy(mpf_class, MKDEV(major, minor));

	cdev_del(&mpf_imx->cdev);
unregister_chrdev:
	unregister_chrdev_region(MKDEV(major, minor), num_devices);

	return rc;
}

static void mpf_cleanup_chrdev(struct mpf_imx *mpf_imx)
{
	int minor;
	int major;
	int num_devices = ARRAY_SIZE(devlist);

	major = MAJOR(mpf_devt);

	for (minor = 0; minor < num_devices; minor++) {
		disable_dma_win(&devlist[minor]);
		device_destroy(mpf_class, MKDEV(major, minor));
	}
	cdev_del(&mpf_imx->cdev);
	unregister_chrdev_region(mpf_devt, num_devices);

	dev_dbg(mpf_imx->dev, "%s: Removed %d device files.\n", __func__,
			num_devices);

	if (!mpf_imx->dev->parent->class) {
		/* We created our own class */
		dev_info(mpf_imx->dev,
				"%s: detroying class %s.\n", __func__,
				mpf_class->name);
		class_destroy(mpf_class);
	}
}


static void mpf_imx_irq_work(struct work_struct *work)
{
	struct mpf_imx_sensor *sensor = IMX_WORK_TO_IMX_SENSOR(work);
	struct fb_allocs *working_buf;
	struct fb_allocs *done_buf;
	struct fb_allocs *ready_buf;
	struct device *dev = sensor->dev;

	mpf_imx_irq_count++;

	if (!atomic_read(&sensor->started)) {
		dev_warn(dev, "%s called though stopped\n", __func__);
		return;
	}

	spin_lock_irq(&sensor->lock);

	/* Actually this should be implemented in the FPGA. */
	sensor->sequence++;

	if (list_empty(&sensor->working_q)) {
		dev_warn(dev, "%s: irq but nothing in working_q\n", __func__);
		spin_unlock_irq(&sensor->lock);
		return;
	}

	done_buf = list_entry(sensor->working_q.next, struct fb_allocs, list);
	done_buf->imx_buf.sequence = sensor->sequence;
	dev_dbg(sensor->dev, "%s: done buf %u\n", __func__,
			done_buf->imx_buf.index);
	/* Add to the done queue. */
	list_del(sensor->working_q.next);
	list_add_tail(&done_buf->list, &sensor->done_q);
	sensor->dma_done = 1;
	wake_up_interruptible(&sensor->sample_buf_waitq);

	/* Enqueue next buffer */
	if (!list_empty(&sensor->ready_q)) {
		ready_buf = list_entry(sensor->ready_q.next,
					 struct fb_allocs, list);
		list_del(sensor->ready_q.next);
		list_add_tail(&ready_buf->list, &sensor->working_q);
	}

	if (!list_empty(&sensor->working_q)) {
		working_buf = list_entry(sensor->working_q.next,
					 struct fb_allocs, list);
		/* start next in working queue */
		spin_unlock_irq(&sensor->lock);
		mpf_imx_start(sensor, working_buf->imx_buf.offset);
		spin_lock_irq(&sensor->lock);
		working_buf->imx_buf.ts = ktime_get_ns();
	}

	spin_unlock_irq(&sensor->lock);
}

static int are_open_sensors_ready(struct mpf_imx *mpf_imx)
{

	if (mpf_imx->sensor_open_mask == 0)
		return 0;

	return ((mpf_imx->sensor_ready_mask & mpf_imx->sensor_open_mask)
					== mpf_imx->sensor_open_mask);
}

/*
 * Start (trigger next) capture if all sensors are ready.
 */
static int imx_start_thread(void *data)
{
	struct mpf_imx *mpf_imx = data;
	int rc;
	ktime_t ts;
	uint64_t ts_diff;
	unsigned int sleep_ms;
	unsigned int max_exp_ms;

	while (!kthread_should_stop()) {
		if (wait_event_interruptible(mpf_imx->start_waitq,
					are_open_sensors_ready(mpf_imx) ||
					kthread_should_stop())) {
			dev_info(mpf_imx->dev,
				"%s: transfer interrupted\n", __func__);
		}

		if (kthread_should_stop())
			break;

		dev_dbg(mpf_imx->dev, "all sensors completed\n");
		ts = ktime_get();
		ts_diff = ktime_to_us(ktime_sub(ts, mpf_imx->start_timestamp));
		dev_dbg(mpf_imx->dev,
			"%s: ts_diff before busy polling %llu us\n",
			__func__, ts_diff);

		rc = mpf_imx_ready_wait(mpf_imx);
		if (rc)
			continue;
		ts = ktime_get();
		ts_diff = ktime_to_us(ktime_sub(ts, mpf_imx->start_timestamp));
		dev_dbg(mpf_imx->dev,
				"%s: ts_diff after busy polling %llu us\n",
				__func__, ts_diff);

		/*
		 * Self-clearing, we cannot check if already started.
		 * There is no global `started` status flag.
		 */
		mpf_imx->start_timestamp = ktime_get();
		iowrite32(MAIN_CTRL_START,
				&mpf_imx->shared_regs->main_ctrl);

		mpf_imx->sensor_ready_mask = 0;
		/*
		 * Do not busy poll longer than necessary.
		 * Prohibition time from trigger to trigger is
		 * 49.22 ms (6152500 × 8 ns).
		 */
		sleep_ms = 49 + mpf_imx->xtrig_delay_ms;
		max_exp_ms = max3(mpf_imx->exposure_ms[0],
				mpf_imx->exposure_ms[1],
				mpf_imx->exposure_ms[2]);
		sleep_ms += max_exp_ms / 2;
		dev_dbg(mpf_imx->dev, "START, max_exp %u, msleep %u\n",
				max_exp_ms, sleep_ms);
		usleep_range(sleep_ms * 1000, sleep_ms * 1000);
	}

	return 0;
}

/*
 * This IRQ handler assumes that there is only one PCI IRQ for all sub-devices.
 */
static irqreturn_t mpf_imx_isr(int irq, void *cookie)
{
	uint32_t status_reg;
	uint32_t ctrl_reg;
	struct mpf_imx_sensor *sensor = cookie;
	struct device *dev = sensor->dev;
	struct imx_ctrl *regs = sensor->imx_ctrl;

	status_reg = ioread32(&regs->status);
	if (!(status_reg & STATUS_IRQ_PENDING)) {
		dev_warn(dev, "not an IMX IRQ, status %08x\n", status_reg);
		return IRQ_NONE;
	}

	if (status_reg & STATUS_IN_PROGRESS) {
		dev_err(dev, "%s: DMA still active, status_reg %08x\n",
				__func__, status_reg);
	}

	ctrl_reg = ioread32(&sensor->mpf_imx->shared_regs->main_ctrl);
	if (ctrl_reg & MAIN_CTRL_CANCEL)
		dev_err(dev, "%s: Cancel bit set %08x\n", __func__, ctrl_reg);

	/* clear IRQ */
	spin_lock(&sensor->lock);
	ctrl_reg = ioread32(&regs->config);
	ctrl_reg |= CONFIG_CLR_IRQ;
	iowrite32(ctrl_reg, &regs->config);
	spin_unlock(&sensor->lock);

	/* check if a stop is in progress */
	if (!atomic_read(&sensor->started)) {
		dev_warn(dev, "%s stop in progress.\n", __func__);
		return IRQ_HANDLED;
	}

	schedule_work(&sensor->irq_work);

	return IRQ_HANDLED;
}

static int request_and_map(struct platform_device *pdev, const char *name,
			   struct resource **res, void __iomem **ptr)
{
	int rc;
	struct device *dev = &pdev->dev;

	*res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (*res == NULL) {
		dev_err(dev, "%s: resource %s not defined\n", __func__, name);
		return -ENODEV;
	}

	*ptr = devm_ioremap_resource(&pdev->dev, *res);
	if (IS_ERR(*ptr)) {
		rc = PTR_ERR(*ptr);
		dev_err(dev, "%s: cannot ioremap %s: %d\n",
				__func__, name, rc);
		return rc;
	}

	return 0;
}

static void iod_move_op(void __iomem *iod_base, uint32_t num_moves,
			uint32_t dir)
{
	iowrite32(num_moves, iod_base + IOD_DELAY_CONTROL_N_MOVES_OFFSET);
	iowrite32(dir, iod_base + IOD_DELAY_CONTROL_DIR_OFFSET);
	iowrite32(0x01, iod_base + IOD_DELAY_CONTROL_START_OFFSET);
}

static void iod_load_op(void __iomem *iod_base, uint32_t bit_enable,
			uint32_t step_presc)
{
	iowrite32(bit_enable, iod_base + IOD_DELAY_CONTROL_BIT_ENABLE_OFFSET);
	iowrite32(step_presc, iod_base + IOD_DELAY_CONTROL_STEP_PRESC_OFFSET);
	iowrite32(0x01, iod_base + IOD_DELAY_CONTROL_LOAD_OFFSET);
}

static uint32_t iod_oor_get(void __iomem *iod_base)
{
	return ioread32(iod_base + IOD_DELAY_CONTROL_OUT_OF_RANGE_OFFSET);
}

static void mpf_iod_delay_init(struct mpf_imx *mpf_imx)
{
	void __iomem *iod_base = mpf_imx->iod_base;
	uint32_t val;

	/* Magic undocumented values */
	iod_load_op(iod_base, 0x3f, 4);
	msleep(20); /* necessary otherwise OOR */

	iod_move_op(iod_base, 130, 0);
	msleep(20); /* necessary otherwise OOR */

	iod_move_op(iod_base, 110, 1);

	val = iod_oor_get(iod_base);
	if (val != 0)
		dev_warn(mpf_imx->dev, "%s: iod_oor %08x\n", __func__, val);
}

static int mpf_imx_probe(struct platform_device *pdev)
{
	int rc;
	struct resource *res;
	struct mpf_imx *mpf_imx;
	void __iomem *base;

	pr_debug("%s\n", __func__);
	mpf_imx = devm_kzalloc(&pdev->dev, sizeof(*mpf_imx), GFP_KERNEL);
	mpf_imx->pdev = pdev;
	mpf_imx->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, mpf_imx);

	if (!pdev->dev.parent) {
		dev_err(&pdev->dev, "parent is NULL\n");
		return -ENODEV;
	}

	mpf_imx->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!mpf_imx->regmap) {
		dev_err(&pdev->dev, "Cannot get regmap.\n");
		return -ENODEV;
	}

	rc = request_and_map(pdev, "imx-shared", &res, &base);
	if (rc)
		return rc;
	mpf_imx->shared_regs = base;
	pr_debug("shared_regs %p, %pR\n", base, res);

	rc = request_and_map(pdev, "imx-0", &res, &base);
	if (rc)
		return rc;
	mpf_imx->imx_ctrl[0] = base;
	pr_debug("imx-0 %p, %pR\n", base, res);

	rc = request_and_map(pdev, "imx-1", &res, &base);
	if (rc)
		return rc;
	mpf_imx->imx_ctrl[1] = base;
	pr_debug("imx-1 %p, %pR\n", base, res);

	rc = request_and_map(pdev, "imx-2", &res, &base);
	if (rc)
		return rc;
	mpf_imx->imx_ctrl[2] = base;
	pr_debug("imx-2 %p, %pR\n", base, res);

	rc = request_and_map(pdev, "imx-iod", &res, &base);
	if (rc)
		return rc;
	mpf_imx->iod_base = base;
	pr_debug("imx-iod %p, %pR\n", base, res);

	/*
	 * Use 32-bit DMA addresses, even if the HW could use 64-bit, to
	 * avoid potential problems.
	 */
	rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(&pdev->dev, "No usable DMA configuration, aborting\n");
		return rc;
	}

	/*
	 * Check for CMA alignment configuration, otherwise we might get
	 * DMA bufs which cross an ATR window boundary.
	 */
	if (get_order(MPF_DMA_WIN_SZ) > CONFIG_CMA_ALIGNMENT) {
		dev_err(&pdev->dev, "CONFIG_CMA_ALIGNMENT  to low, need %u\n",
				get_order(MPF_DMA_WIN_SZ));
		return -EINVAL;
	}

	rc = mpf_init_chrdev(mpf_imx);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: mpf_init_chrdev failed: %d\n", __func__, rc);
		return rc;
	}

	mpf_imx_request_irq(mpf_imx);

	mpf_iod_delay_init(mpf_imx);

	/* Power-on sensors */
	mpf_imx->regulator = devm_regulator_get(&pdev->dev, "V-imx");
	if (IS_ERR(mpf_imx->regulator)) {
		dev_err(&pdev->dev, "Failed to get IMX regulator\n");
		rc = PTR_ERR(mpf_imx->regulator);
		mpf_imx->regulator = NULL;
		/*
		 * XXX: we do not return an error, because sensors
		 * might be switched on by other means.
		 * In a final release we might return an error.
		 */
		rc = 0;
	}
	rc = regulator_enable(mpf_imx->regulator);
	if (rc)
		dev_err(&pdev->dev, "Failed to enable IMX regulator\n");
	/*
	 * XXX: delay time after reset not documented in FPGA documentation,
	 * perhaps not needed.
	 */
	msleep(200);
	iowrite32(MAIN_CTRL_RESET, &mpf_imx->shared_regs->main_ctrl);
	iowrite32(IMX_DMA_TIMEOUT_VAL, &mpf_imx->shared_regs->rx_timeout);
	iowrite32(IMX_XTRIG_DELAY_VAL, &mpf_imx->shared_regs->xtrig_delay);
	mpf_imx->xtrig_delay_ms = MPF_TICKS_TO_MS(IMX_XTRIG_DELAY_VAL);


	/*
	 * Compatiblity check. Newer FPGA versions have an 2 GB AXI to PCI
	 * window and the translation is setup in mpf.ko.
	 * Check if the window is enabled. If not setup and ATR window
	 * for every stream (as in older versions).
	 */
	large_pci_win = is_pci_win_setup(mpf_imx);

	init_waitqueue_head(&mpf_imx->start_waitq);

	mpf_imx->start_thread = kthread_run(imx_start_thread, mpf_imx,
			"imx_start_thread");
	if (IS_ERR(mpf_imx->start_thread)) {
		rc = PTR_ERR(mpf_imx->start_thread);
		dev_err(mpf_imx->dev, "Couldn't create kthread (%d)\n", rc);
		return rc;	/* XXX: do cleanup */
	}

	pr_debug("%s done\n", __func__);
	return rc;
}

static int mpf_imx_remove(struct platform_device *pdev)
{
	struct mpf_imx *mpf_imx = platform_get_drvdata(pdev);

	dev_dbg(mpf_imx->dev, "%s\n", __func__);

	wake_up_interruptible(&mpf_imx->start_waitq);
	kthread_stop(mpf_imx->start_thread);

	mpf_cleanup_chrdev(mpf_imx);

	regulator_disable(mpf_imx->regulator);

	return 0;
}

static struct platform_driver mpf_imx_driver = {
	.probe		= mpf_imx_probe,
	.remove		= mpf_imx_remove,
	.driver		= {
		.name	= "mpf-imx",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(mpf_imx_driver);

MODULE_AUTHOR("Michael Brandt <michael.brandt@devtwig.se>");
MODULE_DESCRIPTION("Alster FPGA IMX module");
MODULE_ALIAS("platform:mpf-imx");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.1.1");

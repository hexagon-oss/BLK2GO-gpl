// SPDX-License-Identifier: GPL-2.0
/*
 * DMA module for Alster EDM module connected to MPF FPGA.
 * There are two sets of common shared registers (ts and misc)
 * and a register set for each EDM channel (edm_ctrl).
 * Every minor device can change the shared registers via IOCTLs.
 */

/* define before device.h to get dev_dbg output */
/* #define DEBUG */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>	/* dma_set_mask */
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>		/* ioremap */
#include <linux/ktime.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>		/* kmalloc */
#include <linux/regmap.h>
#include <linux/uaccess.h>	/* copy_to_user */

/* ioctl and others; shared with user space */
#include "mpf_edm_dev.h"

#define EDM_TIMEOUT	2000	/* in ms */
#define EDM_TIMEOUT_VAL	(EDM_TIMEOUT * 1000 * 125) /* in 125 MHz clock ticks */

#define DRIVER_NAME "mpf_edm"
#define DRIVER_VERSION "1.2"
#define CLASS_NAME  "mpf_edm"

/* IRQ status and clear register */
#define ISTATUS_HOST_OFFS 0x018c

/* PCI slave window size AXI4 interconnect, see FPGA address layout */
#define MPF_PCI_SLAVE_WIN_SZ	SZ_16M
/* DMA window size, subwindows ATR0 to ATR3 */
#define MPF_DMA_WIN_SZ	SZ_1M
/* PCIE0 slave window 0 */
#define ATR0_AXI4_SLV0_SRCADDR_PARAM 0x0800
#define ATR0_AXI4_SLV0_SRC_ADDR      0x0804
#define ATR0_AXI4_SLV0_TRSL_ADDR_LSB 0x0808
#define ATR0_AXI4_SLV0_TRSL_ADDR_UDW 0x080c
#define ATR0_AXI4_SLV0_TRSL_PARAM    0x0810

/* list of DMA buffers */
struct fb_allocs {
	struct list_head list;
	struct edm_buf edm_buf;
};

/*
 * This is the delay from capturing the timestamp in EDM FPGA to
 * reception of frame in MAIN FPGA. Delay from capturing in EDM FPGA to
 * sending is 65 x 8ns = 520ns. Add intrinsic delay from copper wires.
 * Delay is expressed in 125MHz clock cycles.
 */
struct ts_regs {
	/* Intrinsic delay from EDM FPGA to Main FPGA */
	uint32_t timestamp_line_delay;
	/* Timestamp offset between EDM FPGA and Main FPGA */
	uint32_t timestamp_offset;
};

/* Per channel registers. */
struct edm_ctrl {
	uint32_t ctrl;
#define EDM_CTRL_ENABLE		BIT(0)
#define EDM_CTRL_DMA_START	BIT(1)
#define EDM_CTRL_DMA_ABORT	BIT(2)
#define EDM_CTRL_INT_CLEAR	BIT(3)
#define EDM_CTRL_TEST_EN	BIT(4)
	uint32_t status;
#define EDM_STAT_DMA		BIT(0)
#define EDM_STAT_INT		BIT(1)
#define EMD_STAT_TIMEOUT	BIT(2)
#define EDM_STAT_ABORT		BIT(3)
	uint32_t timeout;	/* in 125 MHz clock cycles, default 0 s */
	uint32_t dma_addr;	/* AXI address for DMA */
	uint32_t dma_len;	/* 16 byte aligned */
};

/* Shared global registers. */
struct misc_regs {
	uint32_t crc_error_counter;
	/*
	 * Error counter for general errors (e.g. wrong type field, wrong lenth
	 * field,...)
	 */
	uint32_t frame_error_counter;
	/* Test data initial value, from which the counter increments */
	uint32_t sa_test_data_init;
	uint32_t fa_test_data_init;
	uint32_t dist_test_data_init;
};

enum {
	EDM_SA_DEVNUM = 0,
	EDM_FA_DEVNUM = 1,
	EDM_DIST_DEVNUM = 2,
};

#define MPF_EDM_N_DEVS	3

/* driver data */
struct mpf_edm {
	struct platform_device *pdev;
	struct device *dev;
	struct cdev cdev;
	struct regmap *regmap;
	struct ts_regs *ts_regs;
	struct edm_ctrl *edm_ctrl[MPF_EDM_N_DEVS];
	struct misc_regs *misc_regs;
};

/* number of data sample DMA buffers per device */
static struct mpf_edm_device {
	const char *name;
	int minor;
	struct device *dev;
	struct edm_ctrl *edm_ctrl;
	struct mpf_edm *mpf_edm;
	spinlock_t lock;
	unsigned int dma_done;
	unsigned int dma_canceled;
	struct mpf_capinfo capinfo;	/* perf and error info */
	unsigned int n_sample_bufs;
	struct fb_allocs sample_buf[MPF_EDM_MAX_BUFS];
	struct dma_pool *dma_pool;
	size_t dma_chunk_size;		/* rounded up frame size */
	struct list_head ready_q;
	struct list_head done_q;
	struct list_head working_q;
	wait_queue_head_t sample_buf_waitq;
	uint32_t sequence;		/* sequence number for sample buf */

	atomic_t		started;
	struct work_struct	irq_work;
	#define EDM_WORK_TO_EDM(w) container_of(w,	\
					    struct mpf_edm_device,	\
					    irq_work)
	int		irq;
	/* workaround if running without IRQ */
	struct hrtimer	timer;
	unsigned long	sampling_frequency;
	ktime_t		period;
} devlist[] = {
	[EDM_SA_DEVNUM] = {
		.name = "edm-sa",
		.n_sample_bufs = MPF_EDM_SA_N_BUFS,
		.dma_chunk_size = MPF_EDM_SA_DMA_SIZE,
		.sampling_frequency = 10,	/* HZ per DMA block */
	},
	[EDM_FA_DEVNUM] = {
		.name = "edm-fa",
		.n_sample_bufs = MPF_EDM_FA_N_BUFS,
		.dma_chunk_size = MPF_EDM_FA_DMA_SIZE,
		.sampling_frequency = 10,	/* HZ */
	},
	[EDM_DIST_DEVNUM] = {
		.name = "edm-dist",
		.n_sample_bufs = MPF_EDM_DIST_N_BUFS,
		.dma_chunk_size = MPF_EDM_DIST_DMA_SIZE,
		.sampling_frequency = 100,	/* HZ */
	},
};

static bool testmode;
module_param(testmode, bool, 0444);
static struct class *mpf_class;	/* from parent */
static dev_t mpf_devt;

/* for debug purposes */
static unsigned int mpf_edm_irq_count;

/*
 * The parent driver setup a 2 GiB 1:1 window (upper 2 GiB) (Alster)
 * or the parent driver setup a 1 GiB 1:1 window (upper 1 GiB) (SWIFD)
 */
static int large_pci_win;	/* transitional, will go away */


static void mpf_edm_irq_work(struct work_struct *work);
static enum hrtimer_restart mpf_edm_hrtimer_trig_handler(struct hrtimer *);
static irqreturn_t mpf_edm_isr(int irq, void *cookie);

/*
 * find matching allocation/mapping info for given dma_addr
 */
static struct edm_buf *find_alloc_info(struct mpf_edm_device *edm,
							uint32_t dma_addr)
{
	int i;

	for (i = 0; i < edm->n_sample_bufs; i++) {
		if (edm->sample_buf[i].edm_buf.offset == dma_addr)
			return &edm->sample_buf[i].edm_buf;
	}

	return NULL;
}

static void mpf_edm_free_bufs(struct mpf_edm_device *edm)
{
	int i;
	dma_addr_t dma_addr;
	void *vaddr;
	size_t size;
	struct device *dev = edm->dev;
	struct dma_pool *pool = edm->dma_pool;

	for (i = 0; i < edm->n_sample_bufs; i++) {
		vaddr = edm->sample_buf[i].edm_buf.vaddr;
		if (vaddr == NULL)
			continue;

		dma_addr = edm->sample_buf[i].edm_buf.offset;
		size = edm->sample_buf[i].edm_buf.size;
		dev_dbg(dev, "%s: freeing: size %zd, vaddr %p, dma_addr %p\n",
			__func__, size, vaddr, (void *)dma_addr);
		if (pool)
			dma_pool_free(pool, vaddr, dma_addr);
		else
			dma_free_coherent(dev, size, vaddr, dma_addr);

		edm->sample_buf[i].edm_buf.vaddr = 0;
	}

	INIT_LIST_HEAD(&edm->ready_q);
	INIT_LIST_HEAD(&edm->working_q);
	INIT_LIST_HEAD(&edm->done_q);
}

static void disable_dma_win(struct mpf_edm_device *edm)
{
	unsigned int regoffs;
	struct mpf_edm *mpf_edm = edm->mpf_edm;

	regoffs = edm->minor * 0x20; /* select corresponding slave window */
	regmap_write(mpf_edm->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			0);
}

static void setup_dma_win(struct mpf_edm_device *edm, uint32_t dma_addr)
{
	struct mpf_edm *mpf_edm = edm->mpf_edm;
	int log2;
	uint32_t reg;
	unsigned int regoffs;

	regoffs = edm->minor * 0x20; /* select corresponding slave window */

	reg = dma_addr & ~(MPF_DMA_WIN_SZ - 1);
	regmap_write(mpf_edm->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_LSB + regoffs,
			reg);
	regmap_write(mpf_edm->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_UDW + regoffs,
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
	reg = ((log2  - 1) << 1) | 1;	/* e.g. 0x2f for a 16 MB window */
	/* get AXI window part, e.g. f68f.0000 -> 008f.0000 */
	dma_addr = dma_addr & (MPF_PCI_SLAVE_WIN_SZ - 1);
	/* select DMA window, each device has a MPF_DMA_WIN_SZ boundary */
	dma_addr &= ~(MPF_DMA_WIN_SZ - 1);	/* 008f.0000 -> 0080.0000 */
	reg |= dma_addr;
	regmap_write(mpf_edm->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			reg);
}

/*
 * Check if AXI to PCI translation was setup in mpf.ko.
 */
static int is_pci_win_setup(struct mpf_edm *mpf_edm)
{
	uint32_t reg;

	regmap_read(mpf_edm->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM, &reg);
	/* check if 2 GB, enabled, AXI src addr 0x8000.00000 */
	if (reg != 0x8000003d && reg != 0xc000003b) {
		dev_warn(mpf_edm->dev,
		"AXI to PCI not setup by mpf.ko, ATR0_AXI4_SLV0_SRCADDR_PARAM %08x\n",
			reg);
		return 0;
	}
	return 1;
}

static int mpf_edm_alloc_bufs(struct mpf_edm_device *edm)
{
	int rc = 0;
	int i;
	dma_addr_t dma_addr;
	void *vaddr;
	struct dma_pool *pool = edm->dma_pool;
	size_t size = edm->dma_chunk_size;

	for (i = 0; i < edm->n_sample_bufs; i++) {
		if (pool) {
			vaddr = dma_pool_alloc(pool, GFP_KERNEL,
					&dma_addr);
		} else {
			vaddr = dma_alloc_coherent(edm->dev, size, &dma_addr,
					GFP_KERNEL);
		}
		if (vaddr == NULL) {
			dev_err(edm->dev,
				"%s: error allocating DMA buf (size=%zd)\n",
					__func__, size);
			rc = -ENOMEM;
			goto err;
		}
		*(unsigned int *)vaddr = 0x12345670 + i; /* test pattern */
		dev_dbg(edm->dev, "dma mapped data %pad is at %p (%zd)\n",
			&dma_addr, vaddr, size);

		edm->sample_buf[i].edm_buf.offset = (off_t) dma_addr;
		edm->sample_buf[i].edm_buf.vaddr = vaddr;
		edm->sample_buf[i].edm_buf.size = size;
		edm->sample_buf[i].edm_buf.index = i;

	}

err:
	return rc;
}

#ifdef DEBUG
static void print_slave_atr(struct mpf_edm_device *edm, unsigned int regnum)
{
	unsigned int regoffs;
	uint32_t reg;
	struct mpf_edm *mpf_edm = edm->mpf_edm;
	struct device *dev = edm->dev;

	regoffs = regnum * 0x20;

	dev_dbg(dev, "Slave ATR# %u\n", regnum);

	regmap_read(mpf_edm->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_SRCADDR_PARAM: %08x\n", regnum, reg);

	regmap_read(mpf_edm->regmap, ATR0_AXI4_SLV0_SRC_ADDR + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_SRC_ADDR     : %08x\n", regnum, reg);

	regmap_read(mpf_edm->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_LSB + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_TRSL_ADDR_LSB: %08x\n", regnum, reg);

	regmap_read(mpf_edm->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_UDW + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_TRSL_ADDR_UDW: %08x\n", regnum, reg);

	regmap_read(mpf_edm->regmap, ATR0_AXI4_SLV0_TRSL_PARAM + regoffs,
			&reg);
	dev_dbg(dev, "ATR%u_AXI4_SLV0_TRSL_PARAM   : %08x\n", regnum, reg);
}

static void print_dma_info(struct mpf_edm_device *mpf_edm)
{
	struct edm_ctrl *regs;
	unsigned long long to;
	struct device *dev = mpf_edm->dev;

	regs = mpf_edm->edm_ctrl;

	to = ioread32(&regs->timeout) * 8ULL; /* 125 MHz = 8 ns */
	to /= 1000000ULL;	/* ms */
	dev_dbg(dev, "timeout: 0x%x (%llu ms)\n",
			ioread32(&regs->timeout), to);
	dev_dbg(dev, "dma_addr: 0x%08x, len 0x%x\n",
			ioread32(&regs->dma_addr), ioread32(&regs->dma_len));
	/*
	 * Take translation window into account
	 * ATR0_AXI4_SLV0_SRCADDR_PARAM
	 */
	print_slave_atr(mpf_edm, 0);
	print_slave_atr(mpf_edm, 1);
	print_slave_atr(mpf_edm, 2);
}
#endif

static void mpf_edm_stop(struct mpf_edm_device *edm)
{
	struct edm_ctrl *regs;
	uint32_t status_reg;
	uint32_t ctrl_reg;
	unsigned long timeout;

	/* Do nothing if already stopped. */
	if (!atomic_read(&edm->started)) {
		dev_dbg(edm->dev, "%s: already stopped\n", __func__);
		return;
	}

	atomic_set(&edm->started, 0);

	regs = edm->edm_ctrl;
	/* clear pending IRQ */
	ctrl_reg = ioread32(&regs->ctrl);
	ctrl_reg |= EDM_CTRL_INT_CLEAR;
	iowrite32(ctrl_reg, &regs->ctrl);

	status_reg = ioread32(&regs->status);
	if (status_reg & EDM_STAT_DMA) {
		dev_dbg(edm->dev, "%s: DMA still active\n", __func__);
#ifdef ABORT_BUG_FIXED
		iowrite32(EDM_CTRL_DMA_ABORT, &regs->ctrl);
		/*
		 * Make sure the abort reached the target and is sync'ed.
		 */
		ctrl_reg = ioread32(&regs->ctrl);
		iowrite32(0, &regs->ctrl);	/* XXX: needed? */
#endif
		timeout = jiffies + HZ;
		while (ioread32(&regs->status) & EDM_STAT_DMA) {
			if (time_after(jiffies, timeout)) {
				/*
				 * This can be a serious problem if the DMA is
				 * still transferring to a buffer which will be
				 * freed and used by the system otherwise.
				 */
				dev_err(edm->dev,
					"%s: DMA still active after abort\n",
					__func__);
				break;
			}
			msleep(20);
		}
	}

	/* workaround when running without IRQ */
	if (!edm->irq)
		hrtimer_cancel(&edm->timer);

	cancel_work_sync(&edm->irq_work);

	/* disable channel */
	iowrite32(0, &regs->ctrl);

	mpf_edm_free_bufs(edm);

	/* XXX: power off EDM HW */
}

/*
 * Start DMA from EDM device to onboard memory.
 * DMA address must be acquired via EDM_IOC_ALLOCBUFS.
 */
static ssize_t mpf_edm_start(struct mpf_edm_device *edm, uint32_t dma_addr)
{
	struct edm_buf *edm_buf;
	struct edm_ctrl *regs;
	struct device *dev = edm->dev;

	/* XXX: check if DMA is already running */

	/*
	 * XXX: does not harm, but is not necessary since we are called only
	 * with the DMA addr of a sample_buf
	 */
	/* check if DMA addr is valid */
	edm_buf = find_alloc_info(edm, dma_addr);
	if (edm_buf == NULL) {
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
	regs = edm->edm_ctrl;
	if (large_pci_win)
		iowrite32(dma_addr, &regs->dma_addr);
	else {
		/* transitional, will be deleted */
		setup_dma_win(edm, dma_addr);
		iowrite32(dma_addr & (MPF_PCI_SLAVE_WIN_SZ - 1),
				&regs->dma_addr);
	}
	iowrite32(edm->dma_chunk_size, &regs->dma_len);

	/* XXX: move to init routine, only need to setup once */
	iowrite32(EDM_TIMEOUT_VAL, &regs->timeout);

#ifdef DEBUG
	print_dma_info(edm);
#endif

	/* start DMA, enable data reception */
	if (testmode) {
		iowrite32(EDM_CTRL_DMA_START | EDM_CTRL_ENABLE
				| EDM_CTRL_TEST_EN, &regs->ctrl);
	} else
		iowrite32(EDM_CTRL_DMA_START | EDM_CTRL_ENABLE, &regs->ctrl);

	return 0;
}


static int mpf_edm_mmap(struct file *filep, struct vm_area_struct *vma)
{
	unsigned long offset;
	int rc;
	struct mpf_edm_device *edm = filep->private_data;
	struct device *dev = edm->dev;
	struct edm_buf *edm_buf;

	dev_dbg(dev,
	"mmap: vm_start: 0x%8.8lx, vm_end: 0x%8.8lx, vm_pgoff: 0x%8.8lx\n",
		vma->vm_start, vma->vm_end, vma->vm_pgoff);

	offset = vma->vm_pgoff << PAGE_SHIFT;

	/* check if it is a DMA mapping */
	edm_buf = find_alloc_info(edm, offset);
	if (edm_buf == NULL)
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

static unsigned int mpf_edm_poll(struct file *filep, poll_table *wait)
{
	struct mpf_edm_device *edm = filep->private_data;
	struct device *dev = edm->dev;
	unsigned int mask = 0;
	unsigned int dma_done;
	unsigned int dma_canceled;

	poll_wait(filep, &edm->sample_buf_waitq, wait);

	dev_dbg(dev, "%s mpf_edm_irq_count %u\n", __func__, mpf_edm_irq_count);

	spin_lock_irq(&edm->lock);
	dma_done = edm->dma_done;
	dma_canceled = edm->dma_canceled;
	spin_unlock_irq(&edm->lock);

	if (!list_empty(&edm->done_q)) {
		mask = POLLIN | POLLRDNORM;
		dev_dbg(dev, "%s: data ready\n", __func__);
	}

	/*
	 * DMA was done, but canceled and filled with dummy data.
	 */
	if (edm->dma_canceled) {
		mask = POLLERR;
		dev_dbg(dev, "%s canceled\n", __func__);
	}

	return mask;
}


/*
 * start sample buffer streaming
 */
static int ioctl_start(struct mpf_edm_device *edm, void __user *udata)
{
	int rc;
	struct device *dev = edm->dev;
	uint32_t write_addr;
	struct fb_allocs *bufs;
	struct edm_ctrl *regs = edm->edm_ctrl;

	/*
	 * XXX: return EBUSY if a capture was already started.
	 */

	dev_dbg(dev, "%s\n", __func__);

	if (list_empty(&edm->ready_q)) {
		dev_err(dev, "no buffer queued\n");
		return -EINVAL;
	}

	bufs = list_entry(edm->ready_q.next, struct fb_allocs, list);
	dev_dbg(dev, "%s: index %u, DMA addr 0x%08zx\n", __func__,
				bufs->edm_buf.index, bufs->edm_buf.offset);

	/* move buffer from ready_q to working_q */
	list_del(edm->ready_q.next);
	list_add_tail(&bufs->list, &edm->working_q);
	write_addr = (ulong) bufs->edm_buf.offset & 0xffffffff;

	spin_lock_irq(&edm->lock);
	edm->dma_done = 0;
	edm->dma_canceled = 0;

	/* clear everything */
	iowrite32(0, &regs->ctrl);
	/* clear pending IRQ */
	iowrite32(EDM_CTRL_INT_CLEAR, &regs->ctrl);

	/* start DMA */
	rc = mpf_edm_start(edm, write_addr);
	if (rc) {
		spin_unlock_irq(&edm->lock);
		return rc;
	}

	bufs->edm_buf.ts = ktime_get_ns();
	dev_dbg(dev, "DMA started, ts %lld\n", bufs->edm_buf.ts);

	/* workaround if not interrupt driven */
	if (!edm->irq) {
		dev_warn(dev, "no IRQ specified, using hrtimer.\n");
		hrtimer_start(&edm->timer, edm->period, HRTIMER_MODE_REL);
	}

	atomic_set(&edm->started, 1);
	spin_unlock_irq(&edm->lock);

	return 0;
}

/*
 * stop sample buffer streaming
 */
static int ioctl_stop(struct mpf_edm_device *edm, void __user *udata)
{
	struct device *dev = edm->dev;

	dev_dbg(dev, "%s\n", __func__);

	mpf_edm_stop(edm);

	return 0;
}

static int ioctl_querybuf(struct mpf_edm_device *edm, void __user *udata)
{
	unsigned int index;
	struct edm_buf edm_buf;
	struct fb_allocs *fb_allocs;
	struct device *dev = edm->dev;

	if (copy_from_user(&edm_buf, udata, sizeof(edm_buf)))
		return -EFAULT;

	index = edm_buf.index;
	if (index >= edm->n_sample_bufs) {
		dev_err(dev, "%s: invalid index %u\n", __func__, index);
		return -EINVAL;
	}

	fb_allocs = &edm->sample_buf[index];
	edm_buf.offset = fb_allocs->edm_buf.offset;
	edm_buf.vaddr = fb_allocs->edm_buf.vaddr;
	edm_buf.size = fb_allocs->edm_buf.size;
	dev_dbg(dev, "%s: index %u. offset %zx, vaddr %p\n", __func__, index,
			edm_buf.offset, edm_buf.vaddr);

	return copy_to_user(udata, &edm_buf, sizeof(edm_buf));
}

static int ioctl_allocbufs(struct mpf_edm_device *edm, void __user *udata)
{
	int i;
	int index;
	int ret;
	struct edm_buf edm_buf;
	size_t size;
	dma_addr_t dma_addr;
	void *vaddr;

	if (copy_from_user(&edm_buf, udata, sizeof(edm_buf)))
		return -EFAULT;

	size = edm->dma_chunk_size;

	mpf_edm_free_bufs(edm);
	mpf_edm_alloc_bufs(edm);
	for (i = 0; i < edm->n_sample_bufs; i++) {
		dma_addr = edm->sample_buf[i].edm_buf.offset;
		vaddr = edm->sample_buf[i].edm_buf.vaddr;
		size = edm->sample_buf[i].edm_buf.size;
		index = edm->sample_buf[i].edm_buf.index;

		edm_buf.offset = (off_t) dma_addr;
		edm_buf.vaddr = vaddr;
		edm_buf.size = size;
		edm_buf.index = index;
	}

	/*
	 * Put buffers on ready_q.
	 */
	for (i = 0; i < edm->n_sample_bufs; i++)
		list_add_tail(&edm->sample_buf[i].list, &edm->ready_q);

	ret = copy_to_user(udata, &edm_buf, sizeof(edm_buf));

	return ret;
}

/*
 * Get statistic and error status
 */
static int ioctl_capinfo(struct mpf_edm_device *edm, void __user *udata)
{
	int ret;
	struct mpf_capinfo capinfo;
	struct misc_regs *misc_regs = edm->mpf_edm->misc_regs;

	/*
	 * Update capture info/statistic. The error counters are global for
	 * all channels. Erroneous data is silently dropped by the FPGA.
	 * There is no error IRQ or status!
	 */
	edm->capinfo.crc_error_counter = misc_regs->crc_error_counter;
	edm->capinfo.frame_error_counter = misc_regs->frame_error_counter;

	ret = copy_to_user(udata, &edm->capinfo, sizeof(capinfo));

	return ret;
}

static int ioctl_qbuf(struct mpf_edm_device *edm, void __user *udata)
{
	int ret = 0;
	unsigned long lock_flags;
	unsigned int index;
	struct edm_buf edm_buf;
	int restart = 0;

	if (copy_from_user(&edm_buf, udata, sizeof(edm_buf)))
		return -EFAULT;

	index = edm_buf.index;

	dev_dbg(edm->dev, "QBUF %u\n", index);

	spin_lock_irqsave(&edm->lock, lock_flags);

	list_add_tail(&edm->sample_buf[index].list, &edm->ready_q);
	/*
	 * If streaming is on and working_q is empty, i.e. run out of buffers,
	 * we have to restart,
	 */
	if (atomic_read(&edm->started)) {
		if (list_empty(&edm->working_q)) {
			/* Warn, since it is a data overflow. */
			dev_warn(edm->dev,
			"Data Overflow detected around sequence# %u.\n",
					edm->sequence);
			restart = 1;
		}
	}

	spin_unlock_irqrestore(&edm->lock, lock_flags);

	if (restart)
		ret = ioctl_start(edm, NULL);

	return ret;
}

static int ioctl_dqbuf(struct mpf_edm_device *edm, void __user *udata)
{
	int ret;
	struct edm_buf edm_buf;
	struct fb_allocs *done_buf;
	struct device *dev = edm->dev;

	if (copy_from_user(&edm_buf, udata, sizeof(edm_buf)))
		return -EFAULT;

	if (list_empty(&edm->done_q)) {
		dev_err(dev, "%s: no buffer in done queue\n", __func__);
		return -EAGAIN;
	}
	done_buf = list_entry(edm->done_q.next, struct fb_allocs, list);
	if (done_buf == NULL) {
		dev_err(dev, "%s: no buffer in done queue\n", __func__);
		return -EAGAIN;
	}
	list_del(edm->done_q.next);

	ret = copy_to_user(udata, &done_buf->edm_buf, sizeof(edm_buf));

	dev_dbg(dev,
		"%s: index %u, offset 0x%zx, val 0x%08x\n", __func__,
		done_buf->edm_buf.index, done_buf->edm_buf.offset,
		*(int *)done_buf->edm_buf.vaddr);
	return ret;
}

static long mpf_edm_ioctl(struct file *filep, unsigned int ioc,
				unsigned long arg)
{
	int ret;
	void __user *udata = (void __user *) arg;
	struct mpf_edm_device *mpf = filep->private_data;

	switch (ioc) {

	case EDM_IOC_ALLOCBUFS:
		ret = ioctl_allocbufs(mpf, udata);
		break;

	case EDM_IOC_QUERYBUF:
		ret = ioctl_querybuf(mpf, udata);
		break;

	case EDM_IOC_START:
		ret = ioctl_start(mpf, udata);
		break;

	case EDM_IOC_STOP:
		ret = ioctl_stop(mpf, udata);
		break;

	case EDM_IOC_CAPINFO:
		ret = ioctl_capinfo(mpf, udata);
		break;

	case EDM_IOC_QBUF:
		ret = ioctl_qbuf(mpf, udata);
		break;

	case EDM_IOC_DQBUF:
		ret = ioctl_dqbuf(mpf, udata);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}


static int mpf_edm_open(struct inode *inode, struct file *file)
{
	struct mpf_edm_device *edm;
	int minor = iminor(inode);

	if (minor > ARRAY_SIZE(devlist))
		return -ENXIO;

	edm = &devlist[minor];
	file->private_data = edm;

	INIT_LIST_HEAD(&edm->ready_q);
	INIT_LIST_HEAD(&edm->working_q);
	INIT_LIST_HEAD(&edm->done_q);
	atomic_set(&edm->started, 0);

	return 0;
}

static int mpf_edm_release(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct mpf_edm_device *edm = &devlist[minor];
	struct device *dev = edm->dev;

	mpf_edm_stop(edm);

	dev_dbg(dev, "%s\n", __func__);
	mpf_edm_free_bufs(edm);

	file->private_data = NULL;

	dev_dbg(dev, "%s done\n", __func__);
	return 0;
}


static const struct file_operations mpf_fileops = {
	.owner            = THIS_MODULE,
	.poll		  = mpf_edm_poll,
	.unlocked_ioctl   = mpf_edm_ioctl,
	.mmap             = mpf_edm_mmap,
	.open             = mpf_edm_open,
	.release          = mpf_edm_release,
};

static void mpf_edm_request_irq(struct mpf_edm *mpf_edm)
{
	int irq;
	int minor;
	int rc;
	int num_devices = ARRAY_SIZE(devlist);
	struct mpf_edm_device *edm;

	for (minor = 0; minor < num_devices; minor++) {
		edm = &devlist[minor];

		irq = platform_get_irq(mpf_edm->pdev, minor);
		if (irq < 0) {
			dev_err(mpf_edm->dev,
				"%s: Could not get IRQ resource %d.\n",
						__func__, minor);
			continue;
		}

		rc = devm_request_threaded_irq(mpf_edm->dev, irq, NULL,
						mpf_edm_isr, 0,
						DRIVER_NAME, edm);
		if (rc) {
			dev_err(mpf_edm->dev,
				"%s: Can't allocate irq %d\n", __func__,
						irq);
			continue;
		}
		edm->irq = irq;
	}
}

static int mpf_init_chrdev(struct mpf_edm *mpf_edm)
{
	int rc;
	int minor, major;
	struct device *device;
	int num_devices = ARRAY_SIZE(devlist);

	mpf_class = mpf_edm->dev->parent->class;
	if (!mpf_class) {
		mpf_class = class_create(THIS_MODULE, CLASS_NAME);
		if (IS_ERR(mpf_class)) {
			dev_err(mpf_edm->dev,
					"%s: cannot create class\n", __func__);
			return PTR_ERR(mpf_class);
		}
	}

	rc = alloc_chrdev_region(&mpf_devt, 0, /* minor start */
				 num_devices, DRIVER_NAME);
	if (rc) {
		dev_err(mpf_edm->dev, "%s: Failed to obtain major/minors.\n",
						__func__);
		return rc;
	}

	major = MAJOR(mpf_devt);
	minor = MINOR(mpf_devt);

	cdev_init(&mpf_edm->cdev, &mpf_fileops);
	mpf_edm->cdev.owner = mpf_fileops.owner;
	rc = cdev_add(&mpf_edm->cdev, MKDEV(major, minor), num_devices);
	if (rc) {
		dev_err(mpf_edm->dev, "%s: Failed to add cdev. Aborting.\n",
					__func__);
		goto unregister_chrdev;
	}

	for (minor = 0; minor < num_devices; minor++) {
		device = device_create(mpf_class,
				       mpf_edm->dev,
				       MKDEV(major, minor),
				       NULL,
				       "%s", devlist[minor].name);
		if (IS_ERR(device)) {
			dev_err(mpf_edm->dev,
			"%s: Failed to create %s-%d device. Aborting.\n",
					 __func__, DRIVER_NAME, minor);
			rc = -ENODEV;
			goto unroll_device_create;
		}
		devlist[minor].dev = device;
		devlist[minor].mpf_edm = mpf_edm;
		devlist[minor].edm_ctrl = mpf_edm->edm_ctrl[minor];

		init_waitqueue_head(&devlist[minor].sample_buf_waitq);
		spin_lock_init(&devlist[minor].lock);

		INIT_WORK(&devlist[minor].irq_work, mpf_edm_irq_work);

		/*
		 * Create DMA coherent memory pool for each device
		 */
		devlist[minor].minor = minor;
		devlist[minor].dma_pool = dmam_pool_create(DRIVER_NAME,
				mpf_edm->dev,
				devlist[minor].dma_chunk_size,
				PAGE_SIZE, /* align */
				MPF_DMA_WIN_SZ); /* boundary */
		if (!devlist[minor].dma_pool) {
			dev_err(mpf_edm->dev,
					"Failed to create DMA pool for %s\n",
					devlist[minor].name);
		}

		/*
		 * workaround if running without IRQ
		 */
		hrtimer_init(&devlist[minor].timer, CLOCK_MONOTONIC,
							HRTIMER_MODE_REL);
		devlist[minor].timer.function = mpf_edm_hrtimer_trig_handler;
		devlist[minor].period = ktime_set(0,
			NSEC_PER_SEC / devlist[minor].sampling_frequency);
	}

	dev_dbg(mpf_edm->dev, "%s: Created %d device files.\n", __func__,
			num_devices);
	return 0;

unroll_device_create:
	minor--;
	for (; minor >= 0; minor--)
		device_destroy(mpf_class, MKDEV(major, minor));

	cdev_del(&mpf_edm->cdev);
unregister_chrdev:
	unregister_chrdev_region(MKDEV(major, minor), num_devices);

	return rc;
}

static void mpf_cleanup_chrdev(struct mpf_edm *mpf_edm)
{
	int minor;
	int major;
	int num_devices = ARRAY_SIZE(devlist);

	major = MAJOR(mpf_devt);

	for (minor = 0; minor < num_devices; minor++) {
		/* Do not change AXI-PCI window if shared with others. */
		if (!large_pci_win)
			disable_dma_win(&devlist[minor]);
		device_destroy(mpf_class, MKDEV(major, minor));
	}
	cdev_del(&mpf_edm->cdev);
	unregister_chrdev_region(mpf_devt, num_devices);

	dev_dbg(mpf_edm->dev, "%s: Removed %d device files.\n", __func__,
			num_devices);

	if (!mpf_edm->dev->parent->class) {
		/* We created our own class */
		dev_info(mpf_edm->dev,
				"%s: detroying class %s.\n", __func__,
				mpf_class->name);
		class_destroy(mpf_class);
	}
}


static void mpf_edm_irq_work(struct work_struct *work)
{
	struct mpf_edm_device *edm = EDM_WORK_TO_EDM(work);
	struct fb_allocs *working_buf;
	struct fb_allocs *done_buf;
	struct fb_allocs *ready_buf;
	struct device *dev = edm->dev;
	struct edm_ctrl *regs = edm->edm_ctrl;
	uint32_t status_reg;
	uint32_t ctrl_reg;

	mpf_edm_irq_count++;

	if (!atomic_read(&edm->started)) {
		dev_warn(dev, "%s called though stopped\n", __func__);
		return;
	}

	status_reg = ioread32(&regs->status);
	if (status_reg & EDM_STAT_DMA) {
		/*
		 * This happens with the hrtimer as IRQ workaround,
		 * and is an error with the real IRQ.
		 */
		dev_err(edm->dev, "%s: DMA still active, status %08x\n",
				__func__, status_reg);
		return;
	}

	spin_lock(&edm->lock);

	if (status_reg & EDM_STAT_INT) {
		dev_warn(dev, "%s: irq stat %08x\n", __func__, status_reg);
		/* clear pending IRQ */
		ctrl_reg = ioread32(&regs->ctrl);
		ctrl_reg |= EDM_CTRL_INT_CLEAR;
		iowrite32(ctrl_reg, &regs->ctrl);
	}

	/* Actually this should be implemented in the FPGA. */
	edm->sequence++;

	if (list_empty(&edm->working_q)) {
		dev_warn(dev, "%s: irq but nothing in working_q\n", __func__);
		spin_unlock(&edm->lock);
		return;
	}

	done_buf = list_entry(edm->working_q.next, struct fb_allocs, list);
	done_buf->edm_buf.sequence = edm->sequence;
	dev_dbg(edm->dev, "%s: done buf %u\n", __func__,
			done_buf->edm_buf.index);
	/* Add to the done queue. */
	list_del(edm->working_q.next);
	list_add_tail(&done_buf->list, &edm->done_q);
	edm->dma_done = 1;
	wake_up_interruptible(&edm->sample_buf_waitq);

	/* Enqueue next buffer */
	if (!list_empty(&edm->ready_q)) {
		ready_buf = list_entry(edm->ready_q.next,
					 struct fb_allocs, list);
		list_del(edm->ready_q.next);
		list_add_tail(&ready_buf->list, &edm->working_q);
	}

	if (!list_empty(&edm->working_q)) {
		working_buf = list_entry(edm->working_q.next,
					 struct fb_allocs, list);
		/* start next in working queue */
		mpf_edm_start(edm, working_buf->edm_buf.offset);
		working_buf->edm_buf.ts = ktime_get_ns();
	}

	spin_unlock(&edm->lock);
}

static enum hrtimer_restart mpf_edm_hrtimer_trig_handler(
		struct hrtimer *timer)
{
	struct mpf_edm_device *edm;

	edm = container_of(timer, struct mpf_edm_device, timer);

	if (!atomic_read(&edm->started)) {
		dev_warn(edm->dev, "%s called though stopped\n", __func__);
		return HRTIMER_NORESTART;
	}

	hrtimer_forward_now(timer, edm->period);

	schedule_work(&edm->irq_work);

	return HRTIMER_RESTART;
}

/*
 * This IRQ handler assumes that there is only one PCI IRQ for all sub-devices.
 */
static irqreturn_t mpf_edm_isr(int irq, void *cookie)
{
	uint32_t status_reg;
	uint32_t ctrl_reg;
	struct mpf_edm_device *edm_dev = cookie;
	struct mpf_edm *mpf_edm = edm_dev->mpf_edm;
	struct device *dev = edm_dev->dev;
	struct edm_ctrl *regs = edm_dev->edm_ctrl;

	status_reg = ioread32(&regs->status);
	if (!(status_reg & EDM_STAT_INT)) {
		/*
		 * XXX: this may happen also if streaming is stopped while
		 * an IRQ is pending.
		 */
		regmap_read(mpf_edm->regmap, ISTATUS_HOST_OFFS, &status_reg);
		dev_warn(dev, "not an EDM IRQ, PCIESS ISTATUS %08x\n",
				status_reg);
		return IRQ_NONE;
	}
	/* clear IRQ */
	spin_lock(&edm_dev->lock);
	ctrl_reg = ioread32(&regs->ctrl);
	ctrl_reg |= EDM_CTRL_INT_CLEAR;
	iowrite32(ctrl_reg, &regs->ctrl);
	spin_unlock(&edm_dev->lock);

	/* check if a stop is in progress */
	if (!atomic_read(&edm_dev->started)) {
		dev_dbg(dev, "%s stop in progress.\n", __func__);
		return IRQ_HANDLED;
	}

	schedule_work(&edm_dev->irq_work);

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

static int mpf_edm_probe(struct platform_device *pdev)
{
	int rc;
	struct resource *res;
	struct mpf_edm *mpf_edm;
	void __iomem *base;

	pr_debug("%s\n", __func__);
	mpf_edm = devm_kzalloc(&pdev->dev, sizeof(*mpf_edm), GFP_KERNEL);
	mpf_edm->pdev = pdev;
	mpf_edm->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, mpf_edm);

	if (!pdev->dev.parent) {
		dev_err(&pdev->dev, "parent is NULL\n");
		return -ENODEV;
	}

	mpf_edm->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!mpf_edm->regmap) {
		dev_err(&pdev->dev, "Cannot get regmap.\n");
		return -ENODEV;
	}

	rc = request_and_map(pdev, "ts", &res, &base);
	if (rc)
		return rc;

	mpf_edm->ts_regs = base;
	pr_debug("ts_regs %p, %pR\n", base, res);

	rc = request_and_map(pdev, "sa", &res, &base);
	if (rc)
		return rc;

	mpf_edm->edm_ctrl[EDM_SA_DEVNUM] = base;
	pr_debug("sa %p, %pR\n", base, res);

	rc = request_and_map(pdev, "fa", &res, &base);
	if (rc)
		return rc;

	mpf_edm->edm_ctrl[EDM_FA_DEVNUM] = base;
	pr_debug("fa %p, %pR\n", base, res);

	rc = request_and_map(pdev, "dist", &res, &base);
	if (rc)
		return rc;

	mpf_edm->edm_ctrl[EDM_DIST_DEVNUM] = base;
	pr_debug("dist %p, %pR\n", base, res);

	rc = request_and_map(pdev, "misc", &res, &base);
	if (rc)
		return rc;

	mpf_edm->misc_regs = base;
	pr_debug("misc %p, %pR\n", base, res);

	/*
	 * Use 32-bit DMA addresses, even if the HW could use 64-bit, to
	 * avoid potential problems.
	 */
	rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(&pdev->dev, "No usable DMA configuration, aborting\n");
		return rc;
	}

	rc = mpf_init_chrdev(mpf_edm);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: mpf_init_chrdev failed: %d\n", __func__, rc);
		return rc;
	}

	mpf_edm_request_irq(mpf_edm);

	/*
	 * Compatiblity check. Newer FPGA versions have an 2 GB AXI to PCI
	 * window and the translation is setup in mpf.ko.
	 * Check if the window is enabled. If not setup an ATR window
	 * for every stream (as in older versions).
	 */
	large_pci_win = is_pci_win_setup(mpf_edm);
	if (!large_pci_win) {
		int minor;

		for (minor = 0; minor < ARRAY_SIZE(devlist); minor++) {
			/*
			 * Disable PCI slave windows we will use until they are
			 * setup. The FPGA might have enabled them by default.
			 * For example if ATR0 is enabled with wrong settings
			 * and the user opens edm-fa first (ATR1), then the DMA
			 * will overwrite system memory.
			 */
			disable_dma_win(&devlist[minor]);
		}
	}


	pr_debug("%s done\n", __func__);
	return rc;
}

static int mpf_edm_remove(struct platform_device *pdev)
{
	struct mpf_edm *mpf_edm = platform_get_drvdata(pdev);

	dev_dbg(mpf_edm->dev, "%s\n", __func__);

	mpf_cleanup_chrdev(mpf_edm);

	return 0;
}

static struct platform_driver mpf_edm_driver = {
	.probe		= mpf_edm_probe,
	.remove		= mpf_edm_remove,
	.driver		= {
		.name	= "mpf-edm",
		.owner = THIS_MODULE,
	},
};

module_platform_driver(mpf_edm_driver);

MODULE_AUTHOR("Michael Brandt <michael.brandt@devtwig.se>");
MODULE_DESCRIPTION("Alster FPGA EDM module");
MODULE_VERSION(DRIVER_VERSION);
MODULE_ALIAS("platform:mpf-edm");
MODULE_LICENSE("GPL v2");

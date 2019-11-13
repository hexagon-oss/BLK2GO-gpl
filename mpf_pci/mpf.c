// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe module for Alster Microsemi Polarfire on main board
 */

/* define before device.h to get dev_dbg output */
/* #define DEBUG */
#include <linux/cdev.h>
#include <linux/delay.h>		/* msleep() */
#include <linux/i2c.h>
#include <linux/i2c-xiic.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/core.h>
#include <linux/mfd/mpf.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pci.h>
#include <linux/pci-aspm.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/spi/spi.h>
#include <linux/spi/xilinx_spi.h>	/* xspi_platform_data */
#include <linux/suspend.h>

#define DRIVER_NAME "mpf"
#define DRIVER_VERSION "1.1.2"
#define CLASS_NAME  "mpf"

/* Offset from BAR0 base to PCIE0 and PCIE1 */
#define MPF_PCIE0_OFFS 0x4000
#define MPF_PCIE1_OFFS 0x8000

/*
 * Interrupt status ISTATUS_HOST defines.
 * Names are according to the Microsemi documentation.
 * The INT_REQUEST and MSI are related.
 * INT_REQUEST_0 corresponds to MSI0 and so on.
 */
#define ISTATUS_INT_REQUEST_SHIFT	24

/* IRQ enable register, shared with other drivers */
#define IMASK_HOST_OFFS	0x0188

/* IRQ status and clear register */
#define ISTATUS_HOST_OFFS 0x018c

#define MPF_NUM_MSI	8
/*
 * With the IRQ expander in the FPGA, the number
 * of IRQ resources is higher than the number of MSI IRQs.
 *
 * Intern PCIESS IRQs have number 0 to 7
 * IRQs connected to MPF_IRQS_0_REG: 32 - 63
 * IRQs connected to MPF_IRQS_1_REG: 64 - 95
 * IRQs connected to MPF_IRQS_2_REG: 96 - 127
 */
#define MPF_IRQS_REG_OFFS 0xb0000	/* relative to mpf->csr (BAR0) */
#define MPF_IRQ_CTRL_EOI_REG_OFFS 0xb0020
#define MPF_NUM_IRQS_REGS 3
#define MPF_IRQS_0_BASE 32
#define MPF_IRQS_1_BASE 64
#define MPF_IRQS_2_BASE 96
#define MPF_IRQS_3_BASE 128
enum {
	/* PCIESS IRQs */
	MPF_MSI_DMA0	= 0,	/* Fix assigned in PCIESS HW */
	MPF_MSI_DMA1	= 1,	/* Fix assigned in PCIESS HW */

	/* MPF_IRQS_0_REG IRQs */
	MPF_MSI_SPI_IAM	 = MPF_IRQS_0_BASE,
	MPF_MSI_SPI_ADIS,
	MPF_IAM_IRQ,
	MPF_ADIS_IRQ,

	/* MPF_IRQS_1_REG IRQs */
	MPF_MSI_EDM_SA	= MPF_IRQS_1_BASE,	/* Slow Axis MSI# */
	MPF_MSI_EDM_FA,				/* Fast Axis MSI# */
	MPF_MSI_EDM_DIST,			/* Distance data MSI# */

	/* MPF_IRQS_2_REG IRQs */
	MPF_IMX0_DMA_IRQ = MPF_IRQS_2_BASE,
	MPF_IMX1_DMA_IRQ,
	MPF_IMX2_DMA_IRQ,
	MPF_IMX0_I2C_IRQ,
	MPF_IMX1_I2C_IRQ,
	MPF_IMX2_I2C_IRQ,
	MPF_TMP108_I2C_IRQ,

	/* MPF_IRQS_3_BASE IRQs */
	MPF_MSI_UART = MPF_IRQS_3_BASE + 1,

	MPF_NUM_IRQ = MPF_MSI_UART + 1
};

/* PCIE0 master window 0 */
#define ATR0_PCIE_WIN0_SRCADDR_PARAM	0x0600
#define ATR0_PCIE_WIN0_SRC_ADDR		0x0604
#define ATR0_PCIE_WIN0_TRSL_ADDR_LSB	0x0608
#define ATR0_PCIE_WIN0_TRSL_ADDR_UDW	0x060c
#define ATR0_PCIE_WIN0_TRSL_PARAM	0x0610

/* PCIE0 slave window 0 */
#define ATR0_AXI4_SLV0_SRCADDR_PARAM	0x0800
#define ATR0_AXI4_SLV0_SRC_ADDR		0x0804
#define ATR0_AXI4_SLV0_TRSL_ADDR_LSB	0x0808
#define ATR0_AXI4_SLV0_TRSL_ADDR_UDW	0x080c
#define ATR0_AXI4_SLV0_TRSL_PARAM	0x0810

/* Peripheral control register. Used in regulator functions. */
#define MPF_PCTRL_REG_OFFS	0xa0000
#define MPF_PCTRL_ADIS_RST	BIT(1)
#define MPF_PCTRL_ADIS_PWR_EN	BIT(3)
#define MPF_PCTRL_IMX_EN_33	BIT(4)
#define MPF_PCTRL_IMX_EN_22	BIT(5)
#define MPF_PCTRL_IMX_EN_12	BIT(6)

enum {
	MPF_CTRL_BAR	= 0,
	MPF_MEM_BAR	= 2,
};

#ifdef CONFIG_PM_SLEEP
static int mpf_pm_notify(struct notifier_block *notify_block,
			     unsigned long mode, void *unused);
#endif /* CONFIG_PM_SLEEP */

/* character device data */
static struct mpfdev {
	const char *name;
	int bar;
	unsigned long start;
	unsigned long len;
} devlist[] = {
	[0] = { "mpf_ctrl", MPF_CTRL_BAR, 0, 0 },
	[1] = { "mpf_mem",  MPF_MEM_BAR,  0, 0 },
};

/* Use fix I2C bus numbers. */
enum {
	MPF_I2C_BUS_IMX0 = 42,
	MPF_I2C_BUS_IMX1 = 43,
	MPF_I2C_BUS_IMX2 = 44,
	MPF_I2C_BUS_TMP108 = 45,
};

/* I2C info for builtin Linux drivers. */
static struct i2c_board_info mpf_tmp108_i2c_info[] = {
	{
		I2C_BOARD_INFO("tmp108", 0x48),
	},
	{
		I2C_BOARD_INFO("tmp108", 0x49),
	},
	{
		I2C_BOARD_INFO("tmp108", 0x4a),
	},
};

static struct xiic_i2c_platform_data mpf_xiic_platform_data = {
	.devices = mpf_tmp108_i2c_info,
	.num_devices = ARRAY_SIZE(mpf_tmp108_i2c_info)
};


/*
 * SPI controller 0, exclusively used for ADIS IMU.
 * Note: the SPI mode is hard-coded in the FPGA and cannot be changed.
 */
static struct spi_board_info mpf_spi_info_adis[] = {
	{
		.modalias	= "adis16465",
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 1000 * 1000,
		.irq = MPF_ADIS_IRQ,	/* changed in mpf_irq_init() */
	},
};

static struct xspi_platform_data mpf_xspi_platform_data_adis = {
	.devices = mpf_spi_info_adis,
	.num_devices = ARRAY_SIZE(mpf_spi_info_adis),
	.num_chipselect = 1,
	.bits_per_word = 8,	/* hard-coded in FPGA */
};

/*
 * SPI controller 1, exclusively used for IAM IMU.
 * Note: the SPI mode is hard-coded in the FPGA and cannot be changed.
 */
/* XXX: modalias will be replaced with IAM IMU driver */
static struct spi_board_info mpf_spi_info_iam[] = {
	{
		.modalias	= "spidev",
		.chip_select	= 0,
		.mode		= SPI_MODE_3,
		.max_speed_hz	= 1000 * 1000,
	},
};

static struct xspi_platform_data mpf_xspi_platform_data_iam = {
	.devices = mpf_spi_info_iam,
	.num_devices = ARRAY_SIZE(mpf_spi_info_iam),
	.num_chipselect = 1,
	.bits_per_word = 8,	/* hard-coded in FPGA */
};

static int mpf_regmap_read(void *context, unsigned int reg, unsigned int *val)
{
	void __iomem *addr;

	addr = context + reg;
	if (reg >= 0x10000) {
		pr_debug("%s: addr %p, reg %x, val %x\n",
				__func__, addr, reg, *val);
		/* different "context", AXI addr not PCIESS */
		addr -= MPF_PCIE0_OFFS;
		pr_debug("%s: new addr %p, reg %x, val %x\n",
				__func__, addr, reg, *val);
	}

	*val = readl(addr);

	pr_debug("%s: addr %p, reg %x, val %x\n", __func__, addr, reg, *val);

	return 0;
}

static int mpf_regmap_write(void *context, unsigned int reg, unsigned int val)
{
	void __iomem *addr;

	addr = context + reg;
	pr_debug("%s: addr %p, reg %x, val %x\n", __func__, addr, reg, val);

	if (reg >= 0x10000) {
		pr_debug("%s: addr %p, reg %x, val %x\n",
				__func__, addr, reg, val);
		/* different "context", AXI addr not PCIESS */
		addr -= MPF_PCIE0_OFFS;
		pr_debug("%s: new addr %p, reg %x, val %x\n",
				__func__, addr, reg, val);
	}
	writel(val, addr);

	return 0;
}

static struct regmap_config mpf_regmap_config = {
	.fast_io = true,
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.reg_read = mpf_regmap_read,
	.reg_write = mpf_regmap_write,
};

static struct resource mpf_dma_resources[] = {
	{
		.start = MPF_MSI_DMA0,
		.end   = MPF_MSI_DMA0,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_MSI_DMA1,
		.end   = MPF_MSI_DMA1,
		.flags = IORESOURCE_IRQ,
	},
};

static const struct resource mpf_fpgavers_resources[] = {
	{
		/* relative to BAR2 */
		.start = 0x00010000,
		.end   = 0x0001003f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource mpf_spi_adis_resources[] = {
	{
		.name  = "mpf-spi",
		.start = MPF_MSI_SPI_ADIS,
		.end   = MPF_MSI_SPI_ADIS,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x00040000,
		.end   = 0x0004003f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource mpf_spi_iam_resources[] = {
	{
		.name  = "mpf-spi",
		.start = MPF_MSI_SPI_IAM,
		.end   = MPF_MSI_SPI_IAM,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x000f0000,
		.end   = 0x000f003f,
		.flags = IORESOURCE_MEM,
	},

};

/* FPGA peripheral control register */
static const struct resource mpf_pctrl_resources[] = {
	{
		/* relative to BAR2 */
		.start = 0x000A0000,
		.end   = 0x000A001f,
		.flags = IORESOURCE_MEM,
	},
};

static const struct resource imx0_i2c_resources[] = {
	{
		.name  = "SI",
		.start = MPF_IMX0_I2C_IRQ,
		.end   = MPF_IMX0_I2C_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x00020000,
		.end   = 0x0002001f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource imx1_i2c_resources[] = {
	{
		.name  = "SI",
		.start = MPF_IMX1_I2C_IRQ,
		.end   = MPF_IMX1_I2C_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x00020020,
		.end   = 0x0002003f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource imx2_i2c_resources[] = {
	{
		.name  = "SI",
		.start = MPF_IMX2_I2C_IRQ,
		.end   = MPF_IMX2_I2C_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x00020040,
		.end   = 0x0002005f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource tmp108_i2c_resources[] = {
	{
		.name  = "SI",
		.start = MPF_TMP108_I2C_IRQ,
		.end   = MPF_TMP108_I2C_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x00020060,
		.end   = 0x0002007f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource mpf_edm_resources[] = {
	{
		.start = MPF_MSI_EDM_SA,
		.end   = MPF_MSI_EDM_SA,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_MSI_EDM_FA,
		.end   = MPF_MSI_EDM_FA,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_MSI_EDM_DIST,
		.end   = MPF_MSI_EDM_DIST,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.name = "ts",	/* timestamp related */
		.start = 0x000c0000,
		.end   = 0x000c0007,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "sa",	/* slow axis angle data */
		.start = 0x000c0008,
		.end   = 0x000c001b,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "fa",	/* fast axis */
		.start = 0x000c001c,
		.end   = 0x000c002f,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "dist",	/* distance data */
		.start = 0x000c0030,
		.end   = 0x000c0043,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "misc",
		.start = 0x000c0044,
		.end   = 0x000c0058,
		.flags = IORESOURCE_MEM,
	},
};

static const struct resource mpf_edm_resources_swifd[] = {
	{
		.start = MPF_MSI_EDM_SA,
		.end   = MPF_MSI_EDM_SA,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_MSI_EDM_FA,
		.end   = MPF_MSI_EDM_FA,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_MSI_EDM_DIST,
		.end   = MPF_MSI_EDM_DIST,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.name = "ts",	/* timestamp related */
		.start = 0x00040000,
		.end   = 0x00040007,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "sa",	/* slow axis angle data */
		.start = 0x00040008,
		.end   = 0x0004001b,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "fa",	/* fast axis */
		.start = 0x0004001c,
		.end   = 0x0004002f,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "dist",	/* distance data */
		.start = 0x00040030,
		.end   = 0x00040043,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "misc",
		.start = 0x00040044,
		.end   = 0x00040058,
		.flags = IORESOURCE_MEM,
	},
};


static const struct resource mpf_imx_resources[] = {
	{
		.start = MPF_IMX0_DMA_IRQ,
		.end   = MPF_IMX0_DMA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_IMX1_DMA_IRQ,
		.end   = MPF_IMX1_DMA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MPF_IMX2_DMA_IRQ,
		.end   = MPF_IMX2_DMA_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.name = "imx-shared",	/* shared, for all sensors */
		.start = 0x00060000,
		.end   = 0x0006001f,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "imx-0",
		.start = 0x00060020,
		.end   = 0x0006002f,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "imx-1",
		.start = 0x00060040,
		.end   = 0x0006005f,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "imx-2",
		.start = 0x00060060,
		.end   = 0x0006006f,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "imx-iod",
		.start = 0x000d0000,
		.end   = 0x000d001f,
		.flags = IORESOURCE_MEM,
	},
};

static const struct resource mpf_uart_resources[] = {
	/*
	 * Do not set this resource until the mpf_uart driver is ready to
	 * handle IRQs. Otherwise the IRQ will stay on forever.
	 */
	{
		.name  = "UART",
		.start = MPF_MSI_UART,
		.end = MPF_MSI_UART,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x000e0000,
		.end   = 0x000e2fff,
		.flags = IORESOURCE_MEM,
	},
	{
		/* relative to BAR2 */
		.name  = "DMA0",
		.start = 0x00000400,
		.end   = 0x0000043f,
		.flags = IORESOURCE_MEM,
	},
	{
		/* relative to BAR2 */
		.name  = "DMA1",
		.start = 0x00000440,
		.end   = 0x0000047f,
		.flags = IORESOURCE_MEM,
	},

};

static const struct resource mpf_uart_resources_swifd[] = {
	{
		.name  = "UART",
		.start = MPF_MSI_UART,
		.end = MPF_MSI_UART,
		.flags = IORESOURCE_IRQ,
	},
	{
		/* relative to BAR2 */
		.start = 0x00090000,
		.end   = 0x00092fff,
		.flags = IORESOURCE_MEM,
	},
};

static const struct mfd_cell mpf_devs[] = {
	{
		.name = "mpf-dma",
		.resources = mpf_dma_resources,
		.num_resources = ARRAY_SIZE(mpf_dma_resources),
	},
	{
		.name = "mpf-fpgavers",
		.id = -1,
		.resources = mpf_fpgavers_resources,
		.num_resources = ARRAY_SIZE(mpf_fpgavers_resources),
	},
	{
		.name = "mpf-spi",
		.id = 42,	/* SPI bus num */
		.resources = mpf_spi_adis_resources,
		.num_resources = ARRAY_SIZE(mpf_spi_adis_resources),
		.platform_data = &mpf_xspi_platform_data_adis,
		.pdata_size = sizeof(mpf_xspi_platform_data_adis),
	},
	{
		.name = "mpf-spi",
		.id = 43,	/* SPI bus num */
		.resources = mpf_spi_iam_resources,
		.num_resources = ARRAY_SIZE(mpf_spi_iam_resources),
		.platform_data = &mpf_xspi_platform_data_iam,
		.pdata_size = sizeof(mpf_xspi_platform_data_iam),
	},
	{
		.name = "mpf-pctrl",
		.resources = mpf_pctrl_resources,
		.num_resources = ARRAY_SIZE(mpf_pctrl_resources),
	},
	{
		.name = "mscc_i2c",
		.id = MPF_I2C_BUS_IMX0,
		.resources = imx0_i2c_resources,
		.num_resources = ARRAY_SIZE(imx0_i2c_resources),
	},
	{
		.name = "mscc_i2c",
		.id = MPF_I2C_BUS_IMX1,
		.resources = imx1_i2c_resources,
		.num_resources = ARRAY_SIZE(imx1_i2c_resources),
	},
	{
		.name = "mscc_i2c",
		.id = MPF_I2C_BUS_IMX2,
		.resources = imx2_i2c_resources,
		.num_resources = ARRAY_SIZE(imx2_i2c_resources),
	},
	{
		.name = "mscc_i2c",
		.id = MPF_I2C_BUS_TMP108,
		.resources = tmp108_i2c_resources,
		.num_resources = ARRAY_SIZE(tmp108_i2c_resources),
		.platform_data = &mpf_xiic_platform_data,
		.pdata_size = sizeof(mpf_xiic_platform_data),
	},
	{
		.name = "mpf-edm",
		.resources = mpf_edm_resources,
		.num_resources = ARRAY_SIZE(mpf_edm_resources),
	},
	{
		.name = "mpf-imx",
		.resources = mpf_imx_resources,
		.num_resources = ARRAY_SIZE(mpf_imx_resources),
	},
	{
		.name = "mpf_uart",
		.resources = mpf_uart_resources,
		.num_resources = ARRAY_SIZE(mpf_uart_resources),
	},
};

static const struct mfd_cell mpf_devs_swifd[] = {
	{
		.name = "mpf-dma",
		.resources = mpf_dma_resources,
		.num_resources = ARRAY_SIZE(mpf_dma_resources),
	},
	{
		.name = "mpf-fpgavers",
		.id = -1,
		.resources = mpf_fpgavers_resources,
		.num_resources = ARRAY_SIZE(mpf_fpgavers_resources),
	},
	{
		.name = "mpf-spi",
		.id = 43,	/* SPI bus num */
		.resources = mpf_spi_iam_resources,
		.num_resources = ARRAY_SIZE(mpf_spi_iam_resources),
		.platform_data = &mpf_xspi_platform_data_iam,
		.pdata_size = sizeof(mpf_xspi_platform_data_iam),
	},
	{
		.name = "mscc_i2c",
		.id = MPF_I2C_BUS_TMP108,
		.resources = tmp108_i2c_resources,
		.num_resources = ARRAY_SIZE(tmp108_i2c_resources),
		.platform_data = &mpf_xiic_platform_data,
		.pdata_size = sizeof(mpf_xiic_platform_data),
	},
	{
		.name = "mpf-edm",
		.resources = mpf_edm_resources_swifd,
		.num_resources = ARRAY_SIZE(mpf_edm_resources_swifd),
	},
	{
		.name = "mpf_uart",
		.resources = mpf_uart_resources_swifd,
		.num_resources = ARRAY_SIZE(mpf_uart_resources_swifd),
	},
};

static struct class *mpf_class;
static dev_t mpf_devt;


static ssize_t mpf_read(struct file *filep, char *buffer, size_t len,
							loff_t *offset)
{
	struct mpf_device *mpf = filep->private_data;
	struct device *dev = mpf->dev;

	dev_info(dev, "%s\n", __func__);

	return 0;
}

static ssize_t mpf_write(struct file *filep, const char *buffer,
						size_t len, loff_t *offset)
{
	struct mpf_device *mpf = filep->private_data;
	struct device *dev = mpf->dev;

	dev_info(dev, "%s\n", __func__);

	return 0;
}

static int mpf_mmap(struct file *filep, struct vm_area_struct *vma)
{
	unsigned long offset;
	int minor;
	int rc;
	struct mpf_device *mpf = filep->private_data;
	struct device *dev = mpf->dev;
	unsigned long mmio_start, mmio_len;

	minor = MINOR(filep->f_dentry->d_inode->i_rdev);
	mmio_start = devlist[minor].start;
	mmio_len = devlist[minor].len;

	dev_dbg(dev, "mmap: mmio_start 0x%8.8lx, mmio_len: 0x%8.8lx\n",
		mmio_start, mmio_len);

	offset = vma->vm_pgoff << PAGE_SHIFT;
	if ((offset + (vma->vm_end - vma->vm_start)) > mmio_len) {
		dev_err(dev, "%s: size error\n", __func__);
		return -EINVAL;
	}

	offset += (unsigned long) mmio_start;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

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

static int mpf_open(struct inode *inode, struct file *file)
{
	struct mpf_device *mpf = container_of(inode->i_cdev,
			struct mpf_device, cdev);
	int minor = iminor(inode);

	if (minor > ARRAY_SIZE(devlist))
		return -ENXIO;

	file->private_data = mpf;

	return 0;
}

static int mpf_release(struct inode *inode, struct file *file)
{

	file->private_data = NULL;

	return 0;
}


static const struct file_operations mpf_fileops = {
	.owner		  = THIS_MODULE,
	.read		  = mpf_read,
	.write		  = mpf_write,
	.mmap		  = mpf_mmap,
	.open		  = mpf_open,
	.release	  = mpf_release,
};

static int mpf_init_chrdev(struct mpf_device *mpf)
{
	int rc;
	int minor, major;
	struct device *device;
	int num_devices = ARRAY_SIZE(devlist);

	rc = alloc_chrdev_region(&mpf_devt, 0, /* minor start */
				 num_devices, DRIVER_NAME);
	if (rc) {
		pr_warn("%s: Failed to obtain major/minors.\n", __func__);
		return rc;
	}

	major = MAJOR(mpf_devt);
	minor = MINOR(mpf_devt);

	cdev_init(&mpf->cdev, &mpf_fileops);
	mpf->cdev.owner = mpf_fileops.owner;
	rc = cdev_add(&mpf->cdev, MKDEV(major, minor), num_devices);
	if (rc) {
		dev_err(mpf->dev, "%s: Failed to add cdev. Aborting.\n",
					__func__);
		goto unregister_chrdev;
	}

	for (minor = 0; minor < ARRAY_SIZE(devlist); minor++) {
		device = device_create(mpf_class,
				       mpf->dev,
				       MKDEV(major, minor),
				       NULL,
				       "%s", devlist[minor].name);
		if (IS_ERR(device)) {
			dev_err(mpf->dev,
			"%s: Failed to create %s device. Aborting.\n",
				 __func__, devlist[minor].name);
			rc = -ENODEV;
			goto unroll_device_create;
		}
	}

	dev_info(mpf->dev, "%s: Created %d device files.\n", __func__,
			num_devices);
	return 0;

unroll_device_create:
	minor--;
	for (; minor >= 0; minor--)
		device_destroy(mpf_class, MKDEV(major, minor));

	cdev_del(&mpf->cdev);
unregister_chrdev:
	unregister_chrdev_region(MKDEV(major, minor), num_devices);

	return rc;
}

static void mpf_cleanup_chrdev(struct mpf_device *mpf)
{
	int minor;
	int major;
	int num_devices = ARRAY_SIZE(devlist);

	major = MAJOR(mpf_devt);

	for (minor = 0; minor < ARRAY_SIZE(devlist); minor++)
		device_destroy(mpf_class, MKDEV(major, minor));
	cdev_del(&mpf->cdev);
	unregister_chrdev_region(mpf_devt, num_devices);

	dev_info(mpf->dev, "%s: Removed %d device files.\n", __func__,
			num_devices);
}

/*
 * Interrupt handling
 */

static void mpf_irq_disable(struct irq_data *data)
{
	uint32_t irq_mask;
	struct mpf_device *mpf = irq_data_get_irq_chip_data(data);
	unsigned int msi;

	/*
	 * hwirq is the is the FPGA IRQ number starting at MPF_IRQS_0_BASE
	 * We must map it back to MSI# 0..7
	 * If hwirq is already between 0 and 7, then it is an internal
	 * PCIESS IRQ, e.g. DMA.
	 */
	/* XXX: we need an reference count, so it will not be disabled
	 * if any of the driver using it is removed.
	 */
	if (data->hwirq < 8) {
		msi = data->hwirq;
	} else if (data->hwirq >= MPF_IRQS_0_BASE) {
		msi = data->hwirq / 32 - 1;
	} else {
		dev_err(mpf->dev,
			"%s: invalid IRQ#: hwirq (FPGA) %lu, msi# %u\n",
				__func__, data->hwirq, msi);
		return;
	}

	dev_info(mpf->dev, "%s: hwirq (FPGA) %lu, msi# %u\n", __func__,
			data->hwirq, msi);

	irq_mask = BIT(data->hwirq) << ISTATUS_INT_REQUEST_SHIFT;
	/* disable IRQ */
	regmap_update_bits(mpf->regmap, IMASK_HOST_OFFS, irq_mask, 0);
	/* clear IRQ */
	regmap_write(mpf->regmap, ISTATUS_HOST_OFFS, irq_mask);
}

static void mpf_irq_enable(struct irq_data *data)
{
	uint32_t irq_mask;
	struct mpf_device *mpf = irq_data_get_irq_chip_data(data);
	unsigned int msi;

	/*
	 * hwirq is the is the FPGA IRQ number starting at MPF_IRQS_0_BASE
	 * We must map it back to MSI# 0..7
	 * If hwirq is already between 0 and 7, then it is an internal
	 * PCIESS IRQ, e.g. DMA.
	 */
	/* XXX: we need an reference count, so it will not be disabled
	 * if any of the driver using it is removed.
	 * Or does the IRQ framework already do that?
	 */
	if (data->hwirq < 8) {
		msi = data->hwirq;
	} else if (data->hwirq >= MPF_IRQS_0_BASE) {
		msi = data->hwirq / 32 - 1;
	} else {
		dev_err(mpf->dev,
			"%s: invalid IRQ#: hwirq (FPGA) %lu, msi# %u\n",
				__func__, data->hwirq, msi);
		return;
	}

	dev_info(mpf->dev, "%s: hwirq (FPGA) %lu, msi# %u\n", __func__,
			data->hwirq, msi);

	irq_mask = BIT(msi) << ISTATUS_INT_REQUEST_SHIFT;
	/* clear IRQ */
	regmap_write(mpf->regmap, ISTATUS_HOST_OFFS, irq_mask);
	/* enable IRQ */
	regmap_update_bits(mpf->regmap, IMASK_HOST_OFFS,
					irq_mask,	/* mask */
					irq_mask);	/* value */
}

static struct irq_chip mpf_irq_chip = {
	.name = "mpf-irq",
	.irq_enable = mpf_irq_enable,
	.irq_disable = mpf_irq_disable,
};

uint32_t msi_to_irqreg(struct mpf_device *mpf, unsigned int hwirq)
{

	uint32_t reg;
	void *irqregs = mpf->csr + MPF_IRQS_REG_OFFS;

	irqregs = irqregs + sizeof(uint32_t) * hwirq;

	reg = ioread32(irqregs);
	dev_dbg(mpf->dev, "hwirq %u, irqreg %08x\n", hwirq, reg);

	return reg;
}

static irqreturn_t mpf_irq_thread(int irq, void *data)
{
	struct mpf_device *mpf = data;
	unsigned int hwirq;
	unsigned int virq;
	uint32_t istatus_host;
	uint32_t irqreg;
	int bit;

	hwirq = irq - mpf->irq_base;	/* get MSI#, 0 - 7 */
	/*
	 * With the IRQ expander we have to read the IRQ latch
	 * register corresponding to this MSI# (hwirq) and invoke the
	 * handler for each bit set.
	 */
	/* save IRQ status */
	regmap_read(mpf->regmap, ISTATUS_HOST_OFFS, &istatus_host);
	/* ACK IRQ */
	regmap_write(mpf->regmap, ISTATUS_HOST_OFFS,
				BIT(hwirq) << ISTATUS_INT_REQUEST_SHIFT);

	irqreg = msi_to_irqreg(mpf, hwirq);
	if (irqreg == 0) {
		/* stray or PCIESS IRQ (DMA0, DMA1) */
		if (istatus_host & BIT(hwirq)) {
			dev_dbg(mpf->dev, "PCIESS IRQ %u\n", hwirq);
			virq = irq_find_mapping(mpf->irq_domain, hwirq);
			if (virq)
				handle_nested_irq(virq);
		} else {
			/*
			 * Do not print for mpf_uart IRQs, it happens too
			 * often. FPGA bug?
			 */
			if (hwirq != 3)
				dev_warn(mpf->dev,
					"stray IRQ %u, ISTATUS_HOST %08x\n",
					hwirq, istatus_host);
		}
		goto eoi;
	}

	while (irqreg) {
		bit = __ffs(irqreg);
		virq = irq_find_mapping(mpf->irq_domain,
				MPF_IRQS_0_BASE + hwirq * 32 + bit);
		dev_dbg(mpf->dev, "%s, irq %u, hwirq %u, virq %u, bit %d\n",
			__func__, irq, hwirq, virq, bit);
		if (virq)
			handle_nested_irq(virq);
		else {
			dev_warn(mpf->dev,
			"no handler for %s, irq %u, hwirq %u, virq %u, bit %d\n",
			__func__, irq, hwirq, virq, bit);
		}
		irqreg &= ~(1 << bit);
	}

eoi:
	/* XXX: debug */
	regmap_read(mpf->regmap, ISTATUS_HOST_OFFS, &istatus_host);
	if (istatus_host & (BIT(hwirq) << ISTATUS_INT_REQUEST_SHIFT))
		dev_warn(mpf->dev, "new MSI before EOI\n");

	iowrite32(BIT(hwirq), mpf->csr + MPF_IRQ_CTRL_EOI_REG_OFFS);

	return IRQ_HANDLED;
}

static int mpf_irq_domain_map(struct irq_domain *d, unsigned int irq,
					irq_hw_number_t hw)
{
	struct mpf_device *mpf = d->host_data;

	pr_info("%s: irq %d, hwirq %lu\n", __func__, irq, hw);

	irq_set_chip_data(irq, mpf);
	irq_set_chip_and_handler(irq, &mpf_irq_chip, handle_level_irq);
	irq_set_nested_thread(irq, 1);
	irq_set_noprobe(irq);

	return 0;
}

static const struct irq_domain_ops mpf_irq_domain_ops = {
	.map = mpf_irq_domain_map,
};


static int mpf_irq_init(struct mpf_device *mpf)
{
	struct pci_dev *pdev = mpf->pdev;
	int rc;
	int nvec;
	int i;

	pci_set_master(pdev);

	nvec = pci_msi_vec_count(pdev);

	if (nvec != MPF_NUM_MSI) {
		dev_warn(&pdev->dev,
			"Expected %d MSI vectors, vector count is %d\n",
			MPF_NUM_MSI, nvec);
	}
	rc = pci_enable_msi_range(pdev, nvec, nvec);
	if (rc < 0) {
		dev_err(&pdev->dev, "pci_enable_msi_range failed %d\n", rc);
		goto stop_master;
	}
	mpf->num_msi_irqs = rc;

	dev_info(&pdev->dev, "msi vector count %d, irq %u\n", nvec, pdev->irq);

	mpf->irq_domain = irq_domain_add_linear(NULL, MPF_NUM_IRQ,
			&mpf_irq_domain_ops, mpf);
	if (!mpf->irq_domain) {
		dev_err(&pdev->dev, "could not created irq domain\n");
		rc = -ENOMEM;
		goto stop_master;
	}

	for (i = 0; i < mpf->num_msi_irqs; i++) {
		/*
		 * Oneshot interrupts keep the irq line masked until the
		 * threaded handler finished.
		 */
		rc = request_threaded_irq(pdev->irq + i, NULL,
			mpf_irq_thread, IRQF_ONESHOT, "mpf-msi", mpf);
		if (rc) {
			dev_err(mpf->dev,
				"failed to register irq=%d; err: %d\n",
				pdev->irq, rc);
			goto fail_irq;
		}
	}

	mpf->irq_base = pdev->irq;

	/* SWIFD has no ADIS IMU, but an Invensense IMU with motion IRQ */
	if (!mpf->is_swifd) {
		/* Fixup SPI IRQs, HW irq to virtual IRQ */
		i = mpf_spi_info_adis[0].irq;
		if (i) {
			i = irq_create_mapping(mpf->irq_domain, i);
			mpf_spi_info_adis[0].irq = i;
			dev_info(mpf->dev, "Fixup ADIS IRQ: new %d\n", i);
		}
	}
	/* XXX: else { fixup SWIFD Invensense IRQ } */

	/* XXX: FPGA bug workaround. */
	iowrite32(0xff, mpf->csr + MPF_IRQ_CTRL_EOI_REG_OFFS);

	return 0;

fail_irq:
	irq_domain_remove(mpf->irq_domain);
stop_master:
	pci_clear_master(pdev);
	return rc;
}

static int mpf_irq_fini(struct mpf_device *mpf)
{
	int i;
	unsigned int virq;
	struct pci_dev *pdev = mpf->pdev;

	dev_info(&pdev->dev, "%s\n", __func__);

	pci_clear_master(pdev);

	for (i = 0; i < mpf->num_msi_irqs; i++) {
		dev_info(&pdev->dev, "%s: free_irq %u\n", __func__,
				pdev->irq + i);
		free_irq(pdev->irq + i, mpf);
	}

	for (i = 0; i < MPF_NUM_IRQ; i++) {
		virq = irq_find_mapping(mpf->irq_domain, i);
		if (virq) {
			pr_info("%s: virq %d\n", __func__, virq);
			irq_dispose_mapping(virq);
		}
	}

	irq_domain_remove(mpf->irq_domain);
	mpf->irq_domain = NULL;

	mpf->num_msi_irqs = 0;

	/*
	 * The kernel will panic if the driver still has requested any irq!
	 * So we cannot call pci_disable_msi() if we used devm_request_irq().
	 * Therefore use request_irq/free_irq, if you want to handle an
	 * IRQ in this driver.
	 */
	pci_disable_msi(pdev);

	return 0;
}

static void mpf_adis_assert_reset(struct mpf_device *mpf)
{
	/* activate reset (low active) */
	regmap_update_bits(mpf->regmap, MPF_PCTRL_REG_OFFS,
					MPF_PCTRL_ADIS_RST, 0);
}

static void mpf_adis_deassert_reset(struct mpf_device *mpf)
{
	regmap_update_bits(mpf->regmap, MPF_PCTRL_REG_OFFS,
				MPF_PCTRL_ADIS_RST, MPF_PCTRL_ADIS_RST);
}

static int mpf_regulator_enable(struct regulator_dev *rdev)
{
	unsigned int enable_mask;
	unsigned int enable_val;
	struct mpf_device *mpf = rdev_get_drvdata(rdev);

	enable_mask = rdev->desc->enable_mask;
	enable_val = rdev->desc->enable_val;
	dev_dbg(mpf->dev, "%s, enable_mask %08x, val %08x\n",
			__func__, enable_mask, enable_val);

	/* Be on the safe side and keep IMU in reset when powering on. */
	if (rdev->desc->id == MPF_PCTRL_ADIS_PWR_EN)
		mpf_adis_assert_reset(mpf);

	regmap_update_bits(mpf->regmap, MPF_PCTRL_REG_OFFS,
				enable_mask, enable_val);

	/* Take it also out of reset */
	if (rdev->desc->id == MPF_PCTRL_ADIS_PWR_EN) {
		usleep_range(1000, 1500);
		mpf_adis_deassert_reset(mpf);
	}

	return 0;
}

static int mpf_regulator_disable(struct regulator_dev *rdev)
{
	unsigned int enable_mask;
	unsigned int disable_val;
	struct mpf_device *mpf = rdev_get_drvdata(rdev);

	enable_mask = rdev->desc->enable_mask;
	disable_val = rdev->desc->disable_val;
	dev_dbg(mpf->dev, "%s, disable_mask %08x, val %08x\n",
			__func__, enable_mask, disable_val);

	/* Be on the safe side and get the IMU into reset when powering off. */
	if (rdev->desc->id == MPF_PCTRL_ADIS_PWR_EN)
		mpf_adis_assert_reset(mpf);

	regmap_update_bits(mpf->regmap, MPF_PCTRL_REG_OFFS,
					enable_mask, disable_val);
	return 0;
}

static int mpf_regulator_is_enabled(struct regulator_dev *rdev)
{
	unsigned int enable_mask;
	unsigned int enable_val;
	uint32_t reg;
	struct mpf_device *mpf = rdev_get_drvdata(rdev);

	dev_dbg(mpf->dev, "%s\n", __func__);
	regmap_read(mpf->regmap, MPF_PCTRL_REG_OFFS, &reg);

	/* Check for reset if ADIS IMU(low active) */
	if (rdev->desc->id == MPF_PCTRL_ADIS_PWR_EN)
		if (!(reg & MPF_PCTRL_ADIS_RST))
			return 0;

	enable_mask = rdev->desc->enable_mask;
	enable_val = rdev->desc->enable_val;

	dev_dbg(mpf->dev, "%s, reg %08x, enable_mask %08x, val %08x\n",
			__func__, reg, enable_mask, enable_val);
	return ((reg & enable_mask) == enable_val);
}

static struct regulator_init_data mpf_regulator_initdata[] = {
	{
		.constraints = {
			.name = "V-adis",
			.always_on = 0,
			.boot_on = 0,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 3300000,
			.max_uV = 3300000,
		},
	},
	{
		.constraints = {
			.name = "V-imx",
			.always_on = 0,
			.boot_on = 0,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.min_uV = 3300000, /* actually 1.2, 2.2 and 3.3 */
			.max_uV = 3300000,
		},
	},
};

static struct regulator_ops mpf_regulator_ops = {
	.enable      = mpf_regulator_enable,
	.disable     = mpf_regulator_disable,
	.is_enabled  = mpf_regulator_is_enabled,
};

static struct regulator_desc mpf_regulator_desc[] = {
	{
		.name = "regulator-dummy",
		.id = MPF_PCTRL_ADIS_PWR_EN, /* use bitpos in pctrl as id */
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 1,
		.owner = THIS_MODULE,
		.ops = &mpf_regulator_ops,
		.enable_mask = MPF_PCTRL_ADIS_PWR_EN,
		.enable_val = MPF_PCTRL_ADIS_PWR_EN,
	},
	{
		.name = "regulator-dummy",
		.id = MPF_PCTRL_IMX_EN_33, /* use bitpos in pctrl as id */
		.type = REGULATOR_VOLTAGE,
		.n_voltages = 1,
		.owner = THIS_MODULE,
		.ops = &mpf_regulator_ops,
		.enable_mask = MPF_PCTRL_IMX_EN_33 |
				MPF_PCTRL_IMX_EN_22 | MPF_PCTRL_IMX_EN_12,
		.enable_val = MPF_PCTRL_IMX_EN_33 |
				MPF_PCTRL_IMX_EN_22 | MPF_PCTRL_IMX_EN_12,
	},
};

static int mpf_regulator_init(struct mpf_device *mpf)
{
	int i;
	int err;
	struct regulator_dev *rdev;
	struct regulator_desc *desc;
	struct regulator_config config = { };

	config.dev = mpf->dev;
	config.driver_data = mpf;

	for (i = 0; i < ARRAY_SIZE(mpf_regulator_desc); i++) {
		config.init_data = &mpf_regulator_initdata[i];
		desc = &mpf_regulator_desc[i];
		rdev = devm_regulator_register(mpf->dev, desc, &config);
		if (IS_ERR(rdev)) {
			err = PTR_ERR(rdev);
			dev_err(mpf->dev,
				"%s: failed to register regulator %s err %d\n",
				__func__, desc->name,
				err);
			return err;
		}
	}

	return 0;
}

static void aspm_disable(struct mpf_device *mpf)
{
	struct pci_dev *parent = mpf->pdev->bus->self;

	pcie_capability_clear_word(mpf->pdev, PCI_EXP_LNKCTL,
		PCI_EXP_LNKCTL_ASPM_L0S | PCI_EXP_LNKCTL_ASPM_L1);
	/*
	 * Both upstream and downstream PCIe components should
	 * have the same ASPM settings.
	 */
	if (parent) {
		pcie_capability_clear_word(parent, PCI_EXP_LNKCTL,
			PCI_EXP_LNKCTL_ASPM_L0S | PCI_EXP_LNKCTL_ASPM_L1);
	}
}


/*
 * Disable AXI ATR windows (AXI to PCI address translation).
 */
static void mpf_axi_atr_disable(struct mpf_device *mpf)
{
	unsigned int i;
	unsigned int regoffs;

	for (i = 0; i < 6; i++) {
		regoffs = i * 0x20;
		regmap_write(mpf->regmap,
				ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs, 0);
	}
}

/**
 * mpf_axi_pci_win_init() - setup AXI to PCI address translation
 *
 * @mpf: The driver structure
 * @slot: The address translation slot (0-5)
 * @axi_dma_addr: source address
 * @pci_dma_addr: destination address
 * @size: window size (power of 2)
 */
static int mpf_axi_pci_win_init(struct mpf_device *mpf, int slot,
		uint32_t axi_dma_addr, uint32_t pci_dma_addr, size_t size)
{
	int log2;
	uint32_t reg;
	unsigned int regoffs;

	regoffs = slot * 0x20;
	/* 0x8000.0000 & ~0x7fff.ffff */
	reg = pci_dma_addr & ~(size - 1);
	if (reg != pci_dma_addr) {
		dev_err(mpf->dev, "not aligned 0x%08x, 0x%08zx\n",
				pci_dma_addr, size);
		return -EINVAL;
	}

	regmap_write(mpf->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_LSB + regoffs,
			reg);
	regmap_write(mpf->regmap, ATR0_AXI4_SLV0_TRSL_ADDR_UDW + regoffs,
			0);
	log2 = fls(size) - 1;	/* find last bit set */
	reg = axi_dma_addr & ~(size - 1);
	reg |= ((log2  - 1) << 1);	/* e.g. 0x2f for a 16 MB window */
	reg |= 1;			/* enable window */
	regmap_write(mpf->regmap, ATR0_AXI4_SLV0_SRCADDR_PARAM + regoffs,
			reg);

	return 0;
}

static int mpf_pcie_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct mpf_device *mpf;
	int bar;
	int devnum;
	int rc;
	struct resource *res;

	/*
	 * It it is necessary to set the class pointer at the beginning,
	 * since it contains after a rmmod/insmod a bogus pointer.
	 * That part of the device structure is not cleared from the framework
	 * (Linux 3.18). The symptom is a kernel panic in dev_printk which
	 * accesses dev->class->name.
	 */
	pdev->dev.class = mpf_class;

	/* check HW */
	dev_info(&pdev->dev, DRIVER_NAME
		 " device found [%04x:%04x][%04x:%04x] (rev %x)\n",
		 (int)pdev->vendor, (int)pdev->device,
		 (int)pdev->subsystem_vendor, (int)pdev->subsystem_device,
		 (int)pdev->revision);

	/* assign PCI resource only once */
	res = &pdev->resource[MPF_CTRL_BAR];
	if (!res->start && res->end) {
		rc = pci_assign_resource(pdev, MPF_CTRL_BAR);
		if (rc) {
			dev_err(&pdev->dev, "cannot assign PCI space %d\n",
					MPF_CTRL_BAR);
			return rc;
		}
	}
	res = &pdev->resource[MPF_MEM_BAR];
	if (!res->start && res->end) {
		rc = pci_assign_resource(pdev, MPF_MEM_BAR);
		if (rc) {
			dev_err(&pdev->dev, "cannot assign PCI space %d\n",
					MPF_MEM_BAR);
			return rc;
		}
	}

	mpf = devm_kzalloc(&pdev->dev, sizeof(*mpf), GFP_KERNEL);
	mpf->pdev = pdev;
	mpf->dev = &pdev->dev;
	mpf->is_swifd = (pdev->subsystem_device == 0x0001);

	pci_set_drvdata(pdev, mpf);

	rc = pci_enable_device(pdev);
	if (rc) {
		dev_err(&pdev->dev, "pci_enable_device failed: %d\n", rc);
		return rc;
	}

	/*
	 * disable ASPM completely as it slows down performance dramatically
	 */
	aspm_disable(mpf);

	/*
	 * Old style pci region mapping, since pcim_iomap_regions()
	 * does not work with 3.18.31. Upon rmmod the mapping is still
	 * present and cat /proc/iomem after rmmod causes a kernel panic.
	 */
	rc = pci_request_region(pdev, MPF_CTRL_BAR, DRIVER_NAME);
	if (rc) {
		dev_err(&pdev->dev, "pci_request_region failed: %d\n", rc);
		return rc;
	}
	mpf->csr = pci_ioremap_bar(pdev, MPF_CTRL_BAR);
	if (!mpf->csr) {
		dev_err(&pdev->dev, "pci_ioremap_bar failed: %d\n", rc);
		return rc;
	}

	mpf->regmap = devm_regmap_init(&pdev->dev, NULL,
						mpf->csr + MPF_PCIE0_OFFS,
						&mpf_regmap_config);
	if (IS_ERR(mpf->regmap)) {
		dev_err(&pdev->dev, "Failed to initialise regmap\n");
		return PTR_ERR(mpf->regmap);
	}

	dev_info(&pdev->dev, "CSR at %pR -> 0x%p\n",
			&pdev->resource[MPF_CTRL_BAR], mpf->csr);
	dev_info(&pdev->dev, "MEM at %pR\n", &pdev->resource[MPF_MEM_BAR]);

	rc = mpf_irq_init(mpf);
	if (rc) {
		dev_err(&pdev->dev, "mpf_irq_init failed\n");
		goto release_region;
	}

	for (devnum = 0; devnum < ARRAY_SIZE(devlist); devnum++) {
		bar = devlist[devnum].bar;
		devlist[devnum].start = pci_resource_start(pdev, bar);
		devlist[devnum].len = pci_resource_len(pdev, bar);
	}

#ifdef CONFIG_PM_SLEEP
	mpf->pm_notify.notifier_call = mpf_pm_notify;
	rc = register_pm_notifier(&mpf->pm_notify);
	if (rc)
		dev_info(&pdev->dev, "register_pm_notifier failed: %d\n", rc);
#endif /* CONFIG_PM_SLEEP */

	rc = mpf_init_chrdev(mpf);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: mpf_init_chrdev failed: %d\n", __func__, rc);
	}

	/*
	 * Disable AXI ATR windows (AXI to PCI address translation).
	 * They might be on by default (FPGA) or previous loaded drivers.
	 */
	mpf_axi_atr_disable(mpf);

	/*
	 * The FPGA decodes the uppper AXI 2 GB as PCI access.
	 * Map upper 2 GiB 1:1, i.e.
	 * AXI 0x8000.0000 + x -> PCI 0x8000.00000 + x.
	 * Change the mapping if your DMA memory is in the lower 2 GB.
	 */
	if (mpf->is_swifd) {
		uint32_t value;
		void __iomem *reg;

		mpf_axi_pci_win_init(mpf, 0, 0xc0000000, 0xc0000000, SZ_1G);
		/*
		 * XXX: workaround for early FGPAs, fix BAR2 init value
		 * to cover the peripheral devices instead of DDR4.
		 * The PCIESS control register bar covers both, PCIESS
		 * and periperals.
		 */
		reg = mpf->csr + ATR0_PCIE_WIN0_TRSL_ADDR_LSB;
		value = ioread32(reg + 0x20 * MPF_CTRL_BAR);
		iowrite32(value, reg + 0x20 * MPF_MEM_BAR);
	} else
		mpf_axi_pci_win_init(mpf, 0, 0x80000000, 0x80000000, SZ_2G);

	/* NOTE: there is no devm_mfd_add_devices() in 3.x, but in >= 4.7 */
	if (mpf->is_swifd) {
		dev_info(&pdev->dev, "adding SWIFD subdevices\n");
		rc = mfd_add_devices(&pdev->dev, 0,
			mpf_devs_swifd, ARRAY_SIZE(mpf_devs_swifd),
			&pdev->resource[MPF_MEM_BAR], 0, mpf->irq_domain);
	} else {
		rc = mfd_add_devices(&pdev->dev, 0,
			mpf_devs, ARRAY_SIZE(mpf_devs),
			&pdev->resource[MPF_MEM_BAR], 0, mpf->irq_domain);
	}
	if (rc)
		dev_warn(&pdev->dev, "error adding subdevices: %d\n", rc);

	if (!mpf->is_swifd) {
		rc = mpf_regulator_init(mpf);
		if (rc) {
			dev_err(&pdev->dev,
			"%s: mpf_regulator_init failed: %d\n", __func__, rc);
		}
	}

	return 0;

release_region:
	pci_release_region(pdev, MPF_CTRL_BAR);

	return rc;
}

static void mpf_pcie_remove(struct pci_dev *pdev)
{
	struct mpf_device *mpf = pci_get_drvdata(pdev);

	mpf_cleanup_chrdev(mpf);

#ifdef CONFIG_PM
#ifdef CONFIG_PM_SLEEP
	unregister_pm_notifier(&mpf->pm_notify);
#endif /* CONFIG_PM_SLEEP */
#endif /* CONFIG_PM */

	/*
	 * NOTE: before Linux v4.3 mfd_remove_devices() does not
	 * reverse the order; as it should. This cause problems
	 * if a device depents on another.
	 */
	mfd_remove_devices(&pdev->dev);
	mpf_irq_fini(mpf);

	pci_iounmap(pdev, mpf->csr);
	pci_release_region(pdev, MPF_CTRL_BAR);

	pci_disable_device(pdev);
}

#ifdef CONFIG_PM_SLEEP

static int mpf_suspend(struct device *dev)
{
	int rc = 0;
	struct pci_dev *pdev = to_pci_dev(dev);

	pr_info("%s\n", __func__);

	/* TODO: how do I bring card in low power state? */
	pci_save_state(pdev);

	/* disable bus mastering */
	pci_clear_master(pdev);
	/* PCI will call pci_save_state(pdev) and pci_prepare_to_sleep(pdev) */

	return rc;
}

static int mpf_resume(struct device *dev)
{
	int rc = 0;
	struct pci_dev *pdev = to_pci_dev(dev);

	pr_info("%s\n", __func__);

	pci_restore_state(pdev);

	/* allow master */
	pci_set_master(pdev);

	return rc;
}

static int mpf_pm_notify(struct notifier_block *notify_block,
			     unsigned long mode, void *unused)
{
	int rc = 0;

	pr_info("%s: mode (%ld)\n", __func__, mode);

	switch (mode) {
	default:
		pr_info("unhandled notify mode %ld\n", mode);
		break;
	}

	pr_info("notification mode %ld: rc (%d)\n", mode, rc);
	return rc;
}

#define MPF_PM_OPS	(&mpf_pm_ops)

static const struct dev_pm_ops mpf_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mpf_suspend, mpf_resume)
};

#else

#define MPF_PM_OPS	NULL

#endif /* CONFIG_PM_SLEEP */

static const struct pci_device_id mpf_pcie_ids[] = {
	/* Microsemi Polarfire */
	{ 0x11aa, 0x1556, PCI_ANY_ID, PCI_ANY_ID },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, mpf_pcie_ids);

static struct pci_driver mpf_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= mpf_pcie_ids,
	.probe		= mpf_pcie_probe,
	.remove		= mpf_pcie_remove,
	.driver		= {
		.owner = THIS_MODULE,
		.pm = MPF_PM_OPS,
	},
};

static int __init mpf_driver_init(void)
{
	int rc;

	mpf_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(mpf_class)) {
		pr_err("%s: cannot create class\n", __func__);
		return PTR_ERR(mpf_class);
	}

	rc = pci_register_driver(&mpf_pci_driver);
	if (rc) {
		pr_err("%s: driver initialisation failed: %d\n",
				DRIVER_NAME, rc);
		class_destroy(mpf_class);
	}

	return rc;
}

static void __exit mpf_driver_exit(void)
{
	pci_unregister_driver(&mpf_pci_driver);

	class_destroy(mpf_class);
}

module_init(mpf_driver_init);
module_exit(mpf_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael Brandt <michael.brandt@devtwig.se>");
MODULE_DESCRIPTION("Alster Microsemi PCIe module");
MODULE_VERSION(DRIVER_VERSION);

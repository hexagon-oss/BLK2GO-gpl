// SPDX-License-Identifier: GPL-2.0

#include <asm/unaligned.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>		/* poll_table */
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#include "adis16465_dev.h"

#define DRIVER_NAME	"adis16465"
#define DRIVER_VERSION	"2.1.5"

/* FIFO size must be a power of two */
#define ADIS_BURST_FIFO_SIZE	4096
#define ADIS_BURST_FIFO_HIGH	3000	/* print message if reached */

/*
 * FPGA peripheral control register, accessed via regmap obtained from mpf.ko
 * since it is shared with other drivers.
 */
#define MPF_PCTRL_REG_OFFS	0xa0000
#define MPF_PCTRL_ADIS_RST	BIT(1)
#define MPF_PCTRL_ADIS_PWR_EN	BIT(3)
#define MPF_PCTRL_ADIS_INT_EN	BIT(8)
#define MPF_PCTRL_ADIS_INT_CLR	BIT(9)
#define MPF_PCTRL_ADIS_SPI_SEL	BIT(10)

#define MPF_PCTRL_STATUS_REG_OFFS	0xa0004
#define MPF_PCTRL_STATUS_ADIS_INT	BIT(2)
#define MPF_PCTRL_STATUS_IMUFIFO_EMPTY	BIT(6)
#define MPF_PCTRL_STATUS_IMUFIFO_AEMPTY	BIT(7)

#define MPF_PCTRL_ADIS_TS_LO_OFFS	0xa0008
#define MPF_PCTRL_ADIS_TS_HI_OFFS	0xa000c

#define MPF_PCTRL_ADIS_SEPWORD1_OFFS	0xa001c
#define MPF_PCTRL_ADIS_SEPWORD2_OFFS	0xa0020
#define MPF_PCTRL_ADIS_FIFO_OFFS	0xa0024

/* The SYNC signal frequency is hard-coded in the FPGA, no control. */
#define MPF_ADIS_SYNC_FREQ	100

/* Hard-coded in the FPGA, FPGA FIFO is 20 data sets deep (100 ms). */
#define IMUFIFO_DATA_SETS_PER_IRQ 10

struct adis16465 {
	struct spi_device	*spi;
	struct miscdevice	miscdev;
	struct dentry		*debugfs_dentry;

	struct mutex		txrx_lock;
	struct spi_message	msg;
	struct spi_transfer	*xfer;
	void			*buffer;
	uint8_t			rx[4];	/* register TX/RX buffers */
	uint8_t			tx[10];

	/* burst message SPI buffer */
	void			*burst_cmd_buffer; /* allocated */
	void			*burst_rx_buffer; /* raw burst data */
	struct spi_message	burst_msg;
	struct spi_transfer	*burst_xfer;

	/* data32 message SPI buffer */
	void			*data32_buffer;	/* allocated */
	void			*data32_rx_buffer; /* rx data */
	struct spi_message	data32_msg;
	struct spi_transfer	*data32_xfer;

	unsigned long		opened;	/* single open policy */
	atomic_t		started;
	int			irq;
	struct kfifo_rec_ptr_1	burst_fifo;
	wait_queue_head_t	burst_queue;
	struct work_struct	irq_work;
	#define ADIS_WORK_TO_ADIS(w) container_of(w,	\
					    struct adis16465,	\
					    irq_work)
	struct hrtimer		timer;
	unsigned long		sampling_frequency;
	ktime_t			period;

	int			data_mode;
	unsigned int		f_sync;	/* SYNC signal frequency */
	/* status info */
	unsigned int		bursts_dropped;	/* buffer overflows */
	size_t			burst_fifo_high; /* high water mark */

	struct regulator	*regulator;
	struct regmap		*mpf_regmap;	/* mfd driver regmap */

	/* FPGA IMU FIFO specific */
	uint32_t		eop_1;		/* first frame end marker */
	uint32_t		eop_2;		/* second frame end marker */
	struct completion	imufifo_completion;
};

/* Activate FPGA IMU FIFO for DATA32 reads, FPGA > v0.4 */
static bool imufifo = true;
module_param(imufifo, bool, 0444);

/* Do some extra checks for HW/SW test runs. */
static bool testmode;
module_param(testmode, bool, 0444);
static uint16_t last_data_cntr = 0;	/* for debug purposes */
static unsigned int dr_isr_count;
static ktime_t dr_ktime_timestamp;	/* TS saved in data ready ISR */

static uint64_t dr_timestamp;	/* TS saved in data ready ISR */

static int adis_ioctl_stop(struct adis16465 *adis, unsigned long arg);
static void adis_spi_complete(void *context);

int adis_write_reg(struct adis16465 *adis, unsigned int reg,
	unsigned int value, unsigned int size)
{
	int ret, i;
	struct spi_message msg;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = adis->tx,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = adis->tx + 2,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = adis->tx + 4,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = adis->tx + 6,
			.bits_per_word = 8,
			.len = 2,
		}, {
			.tx_buf = adis->tx + 8,
			.bits_per_word = 8,
			.len = 2,
		},
	};

	mutex_lock(&adis->txrx_lock);

	spi_message_init(&msg);

	switch (size) {
	case 4:
		adis->tx[8] = ADIS_WRITE_REG(reg + 3);
		adis->tx[9] = (value >> 24) & 0xff;
		adis->tx[6] = ADIS_WRITE_REG(reg + 2);
		adis->tx[7] = (value >> 16) & 0xff;
		/* fall through */
	case 2:
		adis->tx[4] = ADIS_WRITE_REG(reg + 1);
		adis->tx[5] = (value >> 8) & 0xff;
		/* fall through */
	case 1:
		adis->tx[2] = ADIS_WRITE_REG(reg);
		adis->tx[3] = value & 0xff;
		break;
	default:
		ret = -EINVAL;
		goto out_unlock;
	}

	xfers[size].cs_change = 0;

	for (i = 1; i <= size; i++)
		spi_message_add_tail(&xfers[i], &msg);

	ret = spi_sync(adis->spi, &msg);
	if (ret) {
		dev_err(&adis->spi->dev,
				"Failed to write register 0x%02X: %d\n",
				reg, ret);
	}

out_unlock:
	mutex_unlock(&adis->txrx_lock);

	return ret;
}

/**
 * adis_write_reg_8() - Write single byte to a register
 * @adis: The adis device
 * @reg: The address of the register to be written
 * @value: The value to write
 */
static inline int adis_write_reg_8(struct adis16465 *adis, unsigned int reg,
	uint8_t val)
{
	return adis_write_reg(adis, reg, val, 1);
}

/**
 * adis_write_reg_16() - Write 2 bytes to a pair of registers
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @value: Value to be written
 */
static inline int adis_write_reg_16(struct adis16465 *adis, unsigned int reg,
	uint16_t val)
{
	return adis_write_reg(adis, reg, val, 2);
}

/**
 * adis_write_reg_32() - write 4 bytes to four registers
 * @adis: The adis device
 * @reg: The address of the lower of the four register
 * @value: Value to be written
 */
static inline int adis_write_reg_32(struct adis16465 *adis, unsigned int reg,
	uint32_t val)
{
	return adis_write_reg(adis, reg, val, 4);
}

/**
 * adis_read_reg() - read size bytes from register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 */
static int adis_read_reg(struct adis16465 *adis, unsigned int reg,
	unsigned int *val, unsigned int size)
{
	struct spi_message msg;
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = adis->tx,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = adis->tx + 2,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = adis->tx + 4,
			.rx_buf = adis->rx,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = adis->rx + 2,
			.bits_per_word = 8,
			.len = 2,
		},
	};

	mutex_lock(&adis->txrx_lock);
	spi_message_init(&msg);

	switch (size) {
	case 4:
		adis->tx[2] = ADIS_READ_REG(reg + 2);
		adis->tx[3] = 0;
		spi_message_add_tail(&xfers[1], &msg);
		/* FALLTHROUGH */
	case 2:
		adis->tx[4] = ADIS_READ_REG(reg);
		adis->tx[5] = 0;
		spi_message_add_tail(&xfers[2], &msg);
		spi_message_add_tail(&xfers[3], &msg);
		break;
	default:
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = spi_sync(adis->spi, &msg);
	if (ret) {
		dev_err(&adis->spi->dev, "Failed to read register 0x%02X: %d\n",
				reg, ret);
		goto out_unlock;
	}

	switch (size) {
	case 4:
		*val = get_unaligned_be32(adis->rx);
		break;
	case 2:
		*val = get_unaligned_be16(adis->rx + 2);
		break;
	}

out_unlock:
	mutex_unlock(&adis->txrx_lock);

	return ret;
}

/**
 * adis_read_reg_16() - read 2 bytes from a 16-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 */
static int adis_read_reg_16(struct adis16465 *adis, unsigned int reg,
	uint16_t *val)
{
	unsigned int tmp;
	int ret;

	ret = adis_read_reg(adis, reg, &tmp, 2);
	*val = tmp;

	return ret;
}

static int adis16465_sync_mode_get(struct adis16465 *adis,
					enum adis_sync_mode *mode)
{
	int rc;
	uint16_t val;

	rc = adis_read_reg_16(adis, ADIS16465_REG_MSC_CTRL, &val);
	if (rc)
		return rc;

	*mode = (val >> MSC_CTRL_SYNC_SHIFT) & MSC_CTRL_SYNC_MASK;

	return rc;
}

/*
 * When the ADIS16465 operates in internal clock mode, the nominal output data
 * rate is equal to 2000/(DEC_RATE + 1).
 *
 * When using the input sync option, in scaled sync mode,
 * the output data rate is equal to (f_SYNC * K_ECSF )/(DEC_RATE + 1)
 * where: f_SYNC is the frequency of the clock signal on the SYNC pin.
 * K_ECSF is the value from the UP_SCALE register.
 * When using direct sync mode and pulse sync mode, K_ECSF = 1.
 */
static int adis16465_get_freq(struct adis16465 *adis)
{
	uint16_t dec_rate;
	uint16_t k_ecsf;	/* external clock scale factor */
	int rc;
	int freq;
	enum adis_sync_mode mode;

	rc = adis_read_reg_16(adis, ADIS16465_REG_DEC_RATE, &dec_rate);
	if (rc)
		return rc;

	/*
	 * Read the mode via the MSC_CTRL register, someone could have changed
	 * it from user space via ADIS_IOC_REG_SET.
	 */
	rc = adis16465_sync_mode_get(adis, &mode);
	if (rc)
		return rc;

	switch (mode) {
	case INTERNAL:
		freq = 2000 / (dec_rate + 1);
		break;
	case DIRECT_SYNC:
	case PULSE_SYNC:
		freq = adis->f_sync / (dec_rate + 1);
		break;
	case SCALED_SYNC:
		rc = adis_read_reg_16(adis, ADIS16465_REG_UP_SCALE, &k_ecsf);
		if (rc)
			return rc;
		freq = k_ecsf * adis->f_sync / (dec_rate + 1);
		break;
	default:
		return -EINVAL;
	}

	return freq;
}

static void adis16465_init_scaled_sync_mode(struct adis16465 *adis)
{
	uint16_t msc;

	/*
	 * set frequency to 200 Hz
	 * f = fSYNC * UP_SCALE / (DEC_RATE + 1)
	 * fSYNC = 100, UP_SCALE := 20
	 * DEC_RATE = 100 * 20 / f - 1
	 */
	adis_write_reg_16(adis, ADIS16465_REG_UP_SCALE, 20);
	adis_write_reg_16(adis, ADIS16465_REG_DEC_RATE, 9);

	adis_read_reg_16(adis, ADIS16465_REG_MSC_CTRL, &msc);
	/* Set DR polarity to high active */
	msc |= MSC_CTRL_DR_POL_HIGH;
	/* Set scaled sync mode */
	msc &= ~(MSC_CTRL_SYNC_MASK << MSC_CTRL_SYNC_SHIFT);
	msc |= (SCALED_SYNC << MSC_CTRL_SYNC_SHIFT);
	adis_write_reg_16(adis, ADIS16465_REG_MSC_CTRL, msc);
}

/*
 * When operating in scaled sync mode (Register MSC_CTRL, Bits[4:2] = 010), the
 * burst read response includes the following registers and value: DIAG_STAT,
 * X_GYRO_OUT, Y_GYRO_OUT, Z_GYRO_OUT, X_ACCL_OUT, Y_ACCL_OUT, Z_ACCL_OUT,
 * TEMP_OUT, TIME_STAMP, and the checksum value.
 * Treat each byte as an independent, unsigned, 8-bit number.
 */
uint16_t adis16465_checksum(uint8_t *burstdata)
{
	int i;
	uint16_t s = 0;

	/* The checksum value is not part of the sum */
	for (i = 0; i < ADIS16465_BURST_DATA_LEN - 2; i++)
		s += burstdata[i];

	return s;
}

/*
 * prepare message to get x,y,z_gyro (32-bit), x,y,z_accel (32-bit),
 * internal temperature (16-bit), data update counter (16-bit).
 */
static int adis16465_prepare_data32_msg(struct adis16465 *adis)
{
	int i;
	unsigned int regs[] = {
		ADIS16465_REG_X_GYRO_LOW,
		ADIS16465_REG_X_GYRO_OUT,
		ADIS16465_REG_Y_GYRO_LOW,
		ADIS16465_REG_Y_GYRO_OUT,
		ADIS16465_REG_Z_GYRO_LOW,
		ADIS16465_REG_Z_GYRO_OUT,
		ADIS16465_REG_X_ACCEL_LOW,
		ADIS16465_REG_X_ACCEL_OUT,
		ADIS16465_REG_Y_ACCEL_LOW,
		ADIS16465_REG_Y_ACCEL_OUT,
		ADIS16465_REG_Z_ACCEL_LOW,
		ADIS16465_REG_Z_ACCEL_OUT,
		ADIS16465_REG_TEMP_OUT,
		ADIS16465_REG_DATA_CNTR,
	};
	unsigned int nregs = ARRAY_SIZE(regs);
	size_t size;
	__be16 *rx;
	__be16 *tx;


	adis->data32_xfer = kcalloc(nregs, sizeof(*adis->data32_xfer),
					GFP_KERNEL);
	if (!adis->data32_xfer)
		return -ENOMEM;

	size =  sizeof(int64_t); /* for alignment */
	size += sizeof(struct adis16465_data32_ts); /* add ts to rx data */
	size += sizeof(uint16_t) * nregs;	/* transmitted regs */
	adis->data32_buffer = kzalloc(size, GFP_KERNEL);
	if (!adis->data32_buffer)
		return -ENOMEM;
	/*
	 * adis->data32_buffer      ts_addr (aligned)   tx_buf
	 * |    rx_buf (aligned)    |                   |
	 * |    |                   |                   |
	 * V    v                   v                   v
	 * | .. | XGLh | XGLl | ... | .. | .. | .. | .. | reg[0] ..
	 */
	rx = adis->data32_buffer;
	rx = PTR_ALIGN(rx, sizeof(int64_t));
	adis->data32_rx_buffer = rx;
	tx = rx + sizeof(struct adis16465_data32_ts)/sizeof(*rx);

	/* prepare nregs SPI messages */
	spi_message_init(&adis->data32_msg);
	adis->data32_xfer[0].tx_buf = &tx[0];
	adis->data32_xfer[0].bits_per_word = 8;
	adis->data32_xfer[0].len = 2;
	adis->data32_xfer[0].cs_change = 1,
	adis->data32_xfer[0].delay_usecs = 16;
	spi_message_add_tail(&adis->data32_xfer[0], &adis->data32_msg);
	for (i = 1; i < nregs; i++) {
		adis->data32_xfer[i].tx_buf = &tx[i];
		adis->data32_xfer[i].rx_buf = &rx[i - 1]; /* rx prev. cmd */
		adis->data32_xfer[i].bits_per_word = 8;
		adis->data32_xfer[i].len = 2;
		adis->data32_xfer[i].cs_change = 1,
		adis->data32_xfer[i].delay_usecs = 16;
		spi_message_add_tail(&adis->data32_xfer[i], &adis->data32_msg);
	}
	adis->data32_xfer[nregs].rx_buf = &rx[nregs - 1]; /* rx prev. cmd */
	adis->data32_xfer[nregs].bits_per_word = 8;
	adis->data32_xfer[nregs].len = 2;
	adis->data32_xfer[nregs].cs_change = 0,
	adis->data32_xfer[nregs].delay_usecs = 16;
	spi_message_add_tail(&adis->data32_xfer[nregs], &adis->data32_msg);

	/* fill transmit buffer with register read commands */
	for (i = 0; i < nregs; i++)
		*tx++ = cpu_to_be16(ADIS_READ_REG(regs[i]) << 8);

	adis->data32_msg.complete = adis_spi_complete;
	adis->data32_msg.context = adis;

	return 0;
}

/*
 * prepare burst read message
 */
static int adis16465_prepare_burst_msg(struct adis16465 *adis)
{
	unsigned int burst_length;
	uint8_t *rx;
	uint8_t *tx;
	size_t size;

	burst_length = ADIS16465_BURST_DATA_LEN;

	adis->burst_xfer = kcalloc(2, sizeof(*adis->burst_xfer), GFP_KERNEL);
	if (!adis->burst_xfer)
		return -ENOMEM;

	/*
	 * adis->burst_cmd_buffer            rx_buf        s64 aligned
	 * |                                 |             | burst_buffer
	 * |                                 |             |
	 * V                                 v             v
	 * | 0x68 | 0x00 | .. | .. | .. | .. | cmd_resp[2] | burst_buffer ..
	 */
	size = sizeof(int64_t) + sizeof(struct adis16465_burst_data_ts);
	adis->burst_cmd_buffer = kzalloc(size, GFP_KERNEL);
	if (!adis->burst_cmd_buffer)
		return -ENOMEM;
	tx = adis->burst_cmd_buffer;
	/* align burst data buffer to 64 bit for timestamp handling */
	rx = PTR_ALIGN(tx + 2, sizeof(int64_t));
	adis->burst_rx_buffer = rx;
	rx -= 2;	/* cmd response buffer */

	tx[0] = 0x68;
	tx[1] = 0x00;

	/*
	 * Both variants below do work, but it might be safer
	 * not to rely on the SPI CS behaviour between messages
	 */
#if 0
	adis->xfer[0].tx_buf = tx;
	adis->xfer[0].bits_per_word = 8;
	adis->xfer[0].len = 2;
	adis->xfer[1].rx_buf = adis->burst_buffer;
	adis->xfer[1].bits_per_word = 8;
	adis->xfer[1].len = burst_length;

	spi_message_init(&adis->msg);
	spi_message_add_tail(&adis->xfer[0], &adis->msg);
	spi_message_add_tail(&adis->xfer[1], &adis->msg);
#else
	/*
	 * The whole message must fit into one xfer, otherwise the Actel
	 * CoreSPI controller may toggle SSEL (depending on CFG_MOT_SSEL).
	 * And according to the ADIS-16465 documentation, SSEL must be active
	 * between the cmd and the burst data.
	 */
	adis->burst_xfer[0].tx_buf = tx;
	adis->burst_xfer[0].rx_buf = rx;
	adis->burst_xfer[0].bits_per_word = 8;
	adis->burst_xfer[0].len = 2 + burst_length;
	spi_message_init(&adis->burst_msg);
	spi_message_add_tail(&adis->burst_xfer[0], &adis->burst_msg);
#endif

	adis->burst_msg.complete = adis_spi_complete;
	adis->burst_msg.context = adis;

	return 0;
}

static int adis16465_insert_data(struct adis16465 *adis,
					const void *buf, size_t count)
{
	int ret;
	size_t fifo_len;

	fifo_len = kfifo_len(&adis->burst_fifo);
	dev_dbg(&adis->spi->dev,
		"inserting burst %zd bytes, avail %d, fifo_len %zd\n",
		count, kfifo_avail(&adis->burst_fifo), fifo_len);

	if (kfifo_avail(&adis->burst_fifo) < count) {
		dev_warn_ratelimited(&adis->spi->dev,
			"started %d, count %zd, avail %d, bursts_dropped %d\n",
				atomic_read(&adis->started), count,
				kfifo_avail(&adis->burst_fifo),
				adis->bursts_dropped);

		adis->bursts_dropped++;
		return -ENOSPC;
	}

	ret = kfifo_in(&adis->burst_fifo, buf, count);

	fifo_len = kfifo_len(&adis->burst_fifo);
	if (fifo_len > adis->burst_fifo_high) {
		adis->burst_fifo_high = fifo_len;
		if (fifo_len >= ADIS_BURST_FIFO_HIGH) {
			dev_info_ratelimited(&adis->spi->dev,
				"new fifo high mark %zd bytes\n", fifo_len);
		}
	}

	wake_up_interruptible(&adis->burst_queue);

	return ret;
}

#ifdef CONFIG_DEBUG_FS

static ssize_t adis16465_show_burst(struct file *file,
		char __user *userbuf, size_t count, loff_t *ppos)
{
	int i;
	int ret;
	struct adis16465 *adis = file->private_data;
	size_t len;
	char buf[128];
	uint8_t *rx;
	int16_t checksum;

	spi_setup(adis->spi);

	ret = spi_sync(adis->spi, &adis->burst_msg);

	if (ret)
		dev_err(&adis->spi->dev, "Failed to read data: %d\n", ret);

	rx = adis->burst_rx_buffer;

	/* dump burst into buffer */
	len = 0;
	for (i = 0; i < ADIS16465_BURST_DATA_LEN; i += 2) {
		len += scnprintf(buf + len, sizeof(buf) - len, "%02x.%02x\n",
				rx[i], rx[i+1]);
	}
	/* Ignore the shift-in values from cmd sequence itself. */
	checksum = adis16465_checksum(rx);

	dev_dbg(&adis->spi->dev, "checksum calc: 0x%x\n", checksum);
	len += scnprintf(buf + len, sizeof(buf) - len, "checksum calc 0x%x\n",
			checksum);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}
static const struct file_operations adis16465_show_burst_fops = {
	.open = simple_open,
	.read = adis16465_show_burst,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static int adis16465_show_x_gyro_low(void *arg, u64 *val)
{
	struct adis16465 *adis16465 = arg;
	u16 reg;
	int ret;

	ret = adis_read_reg_16(adis16465, ADIS16465_REG_X_GYRO_LOW,
		&reg);
	if (ret < 0)
		return ret;

	*val = reg;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adis16465_show_x_gyro_low_fops,
	adis16465_show_x_gyro_low, NULL, "%llu\n");

static int adis16465_show_msc_ctrl(void *arg, u64 *val)
{
	struct adis16465 *adis16465 = arg;
	u16 reg;
	int ret;

	ret = adis_read_reg_16(adis16465, ADIS16465_REG_MSC_CTRL,
		&reg);
	if (ret < 0)
		return ret;

	*val = reg;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adis16465_msc_ctrl_fops,
	adis16465_show_msc_ctrl, NULL, "%llu\n");

static int adis16465_show_diag_stat(void *arg, u64 *val)
{
	struct adis16465 *adis16465 = arg;
	u16 reg;
	int ret;

	ret = adis_read_reg_16(adis16465, ADIS16465_REG_DIAG_STAT,
		&reg);
	if (ret < 0)
		return ret;

	*val = reg;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adis16465_diag_stat_fops,
	adis16465_show_diag_stat, NULL, "%llu\n");

static int adis16465_show_product_id(void *arg, u64 *val)
{
	struct adis16465 *adis16465 = arg;
	u16 prod_id;
	int ret;

	ret = adis_read_reg_16(adis16465, ADIS16465_REG_PROD_ID,
		&prod_id);
	if (ret < 0)
		return ret;

	*val = prod_id;

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(adis16465_product_id_fops,
	adis16465_show_product_id, NULL, "%llu\n");

static int adis16465_debugfs_init(struct adis16465 *adis)
{

	adis->debugfs_dentry = debugfs_create_dir("adis16465", NULL);
	debugfs_create_file("product_id", 0444,
		adis->debugfs_dentry, adis,
		&adis16465_product_id_fops);

	debugfs_create_file("diag_stat", 0444,
		adis->debugfs_dentry, adis,
		&adis16465_diag_stat_fops);

	debugfs_create_file("msc_ctrl", 0444,
		adis->debugfs_dentry, adis,
		&adis16465_msc_ctrl_fops);

	debugfs_create_file("x_gyro_low", 0444,
		adis->debugfs_dentry, adis,
		&adis16465_show_x_gyro_low_fops);

	debugfs_create_file("burst", 0444,
		adis->debugfs_dentry, adis,
		&adis16465_show_burst_fops);

	return 0;
}
#else

static int adis16465_debugfs_init(struct adis16465 *adis)
{
	return 0;
}

#endif /* CONFIG_DEBUG_FS */


static void convert_and_insert_data(struct adis16465 *adis)
{
	int i;
	int ret;
	uint16_t *rx;
	size_t record_size;
	uint64_t *ts_addr;
	uint16_t data_cntr;

	if (adis->data_mode == DATA32) {
		rx = adis->data32_rx_buffer;
		record_size = sizeof(struct adis16465_data32_ts);
		ts_addr = &((struct adis16465_data32_ts *)rx)->timestamp;
	} else {
		rx = adis->burst_rx_buffer;
		record_size = sizeof(struct adis16465_burst_data_ts);
		ts_addr = &((struct adis16465_burst_data_ts *)rx)->timestamp;
	}

	/*
	 * The DR timestamp was saved in the DR IRQ handler.
	 * If the DR IRQ comes faster than we can read out data via SPI,
	 * then the sample frequency is too high.
	 * XXX: add a mechanism to detect it.
	 */
	if (!adis->irq)
		*ts_addr = ktime_get_ns();
	else
		*ts_addr = dr_timestamp;

	/* convert be16 data */
	if (adis->data_mode != BURSTRAW) {
		for (i = 0; (void *)&rx[i] < (void *)ts_addr; i++)
			rx[i] = get_unaligned_be16(&rx[i]);
	}

	if (testmode) {
		/* debug, check data_cntr consistency */
		data_cntr = rx[13];	/* for DATA32 mode */
		if (last_data_cntr) {
			if (data_cntr - last_data_cntr > 1) {
				dev_info(&adis->spi->dev,
						"data_cntr %04x, last %04x\n",
						data_cntr, last_data_cntr);
			}
		}
		last_data_cntr = data_cntr;
	}

	ret = adis16465_insert_data(adis, rx, record_size);
	if (ret != record_size)
		dev_err(&adis->spi->dev, "Failed to insert data: %d\n", ret);
}

static void adis16465_irq_work(struct work_struct *work)
{
	int ret;
	struct adis16465 *adis = ADIS_WORK_TO_ADIS(work);

	if (!atomic_read(&adis->started)) {
		dev_warn(&adis->spi->dev, "%s called though stopped\n",
			__func__);
		return;
	}

	dev_dbg(&adis->spi->dev, "%s calling spi_sync\n", __func__);
	if (adis->data_mode == DATA32)
		ret = spi_sync(adis->spi, &adis->data32_msg);
	else
		ret = spi_sync(adis->spi, &adis->burst_msg);

	dev_dbg(&adis->spi->dev, "%s spi_sync done\n", __func__);
	if (ret)
		dev_err(&adis->spi->dev, "Failed to read data: %d\n", ret);

	convert_and_insert_data(adis);
}


static enum hrtimer_restart adis16465_hrtimer_trig_handler(
		struct hrtimer *timer)
{
	struct adis16465 *adis;

	adis = container_of(timer, struct adis16465, timer);

	if (!atomic_read(&adis->started)) {
		dev_warn(&adis->spi->dev, "%s called though stopped\n",
			__func__);
		return HRTIMER_NORESTART;
	}

	hrtimer_forward_now(timer, adis->period);

	schedule_work(&adis->irq_work);

	return HRTIMER_RESTART;
}

/**
 * adis16465_dr_irq_enable() - Enable data ready IRQ
 * @adis: The adis device
 */
static inline void adis16465_dr_irq_enable(struct adis16465 *adis)
{
	/*
	 * The ADIS16465 has no control bit to disable DR activity.
	 * We must enable/disable the interrupt in the FPGA.
	 */
	regmap_update_bits(adis->mpf_regmap, MPF_PCTRL_REG_OFFS,
			MPF_PCTRL_ADIS_INT_EN, MPF_PCTRL_ADIS_INT_EN);
}

/**
 * adis16465_dr_irq_disable() - disable data ready IRQ
 * @adis: The adis device
 */
static inline void adis16465_dr_irq_disable(struct adis16465 *adis)
{
	/*
	 * The ADIS16465 has no control bit to disable DR activity.
	 * We must enable/disable the interrupt in the FPGA.
	 */
	regmap_update_bits(adis->mpf_regmap, MPF_PCTRL_REG_OFFS,
				MPF_PCTRL_ADIS_INT_EN, 0);
}

static inline void adis16465_dr_irq_clr(struct adis16465 *adis)
{
	regmap_update_bits(adis->mpf_regmap, MPF_PCTRL_REG_OFFS,
			MPF_PCTRL_ADIS_INT_CLR, MPF_PCTRL_ADIS_INT_CLR);
}

/*
 * spi_async completion handler
 */
static void adis_spi_complete(void *context)
{
	struct adis16465 *adis = context;
	ktime_t ts;
	uint64_t ts_diff;

	dev_dbg(&adis->spi->dev, "%s, status %d\n", __func__,
			adis->burst_msg.status);

	if (!atomic_read(&adis->started)) {
		dev_warn(&adis->spi->dev, "%s called though stopped\n",
			__func__);
		return;
	}

	convert_and_insert_data(adis);

	adis16465_dr_irq_enable(adis);

	if (testmode) {
		ts = ktime_get();
		ts_diff = ktime_to_us(ktime_sub(ts, dr_ktime_timestamp));
		if (dr_isr_count && ts_diff > 4000) {
			dev_info(&adis->spi->dev,
				"%s: ts_diff %llu us, %u\n", __func__,
				ts_diff, dr_isr_count);
		}
	}
}

/*
 * sync to FPGA IMU FIFO frame delimiter
 */
static int imufifo_sync(struct adis16465 *adis)
{
	uint32_t eop;
	uint32_t status;
	unsigned int trials = 20;

	regmap_read(adis->mpf_regmap, MPF_PCTRL_STATUS_REG_OFFS, &status);
	while (!(status & MPF_PCTRL_STATUS_IMUFIFO_EMPTY) && --trials) {
		/* frame delimiter 1 */
		regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &eop);
		if (eop != adis->eop_1) {
			dev_dbg(&adis->spi->dev,
					"%s: not synced , got %08x\n",
					__func__, eop);
			continue;
		}

		/* frame delimiter 2 */
		regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &eop);
		if (eop == adis->eop_2) {
			dev_dbg(&adis->spi->dev,
					"%s: synced , got %08x\n",
					__func__, eop);
			return 0;
		}
		regmap_read(adis->mpf_regmap,
				MPF_PCTRL_STATUS_REG_OFFS, &status);
	}

	dev_info(&adis->spi->dev, "%s: sync failed, status %08x\n", __func__,
			status);
	return -1;
}

static void dump_rx_buf(struct adis16465 *adis)
{
	uint32_t *rx;
	int i;

	rx = adis->data32_rx_buffer;
	for (i = 0; i < 10; i++) {
		dev_info(&adis->spi->dev, "%d: %08x\n", i, *rx++);
	}
}

/*
 * Read a IMU data set from the FPGA and insert it into the kfifo.
 * The FIFO contains following data, all as 32-bit words.
 * The data is already in little endian byte order.
 *  1. x_gyro
 *  2. y_gyro
 *  3. z_gyro
 *  4. x_accel
 *  5. y_accel
 *  6. z_accel
 *  7. (adis_ts << 16) | temperature
 *  8. data_cntr
 *  9. timestamp low
 * 10. timestamp high
 * 11. frame delimiter 1
 * 12. frame delimiter 2
 */
static void imufifo_read(struct adis16465 *adis)
{
	int i;
	int ret;
	size_t record_size = sizeof(struct adis16465_data32_ts);
	uint16_t temp_out, data_cntr;
	uint32_t eop;
	uint32_t *rx;

	rx = adis->data32_rx_buffer;
	for (i = 0; i < 6; i++) {
		regmap_read(adis->mpf_regmap,
				MPF_PCTRL_ADIS_FIFO_OFFS, &rx[i]);
		dev_dbg(&adis->spi->dev, "%s: got %08x\n",
				__func__, rx[i]);
	}

	/* temp_out is 16 bit, but FIFO datum is 32 bit */
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &rx[6]);
	dev_dbg(&adis->spi->dev, "%s: got temp %08x\n", __func__, rx[6]);
	temp_out = rx[6] & 0xffff;

	/* 16-bit ADIS TS register 0x1e, new with FPGA > 0.5.0 */
	rx[7] = (rx[6] >> 16) & 0xffff;

	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &rx[6]);
	dev_dbg(&adis->spi->dev, "%s: got cntr %08x\n", __func__, rx[6]);
	/* data_cntr is 16 bit, but FIFO datum is 32 bit */
	data_cntr = rx[6] & 0xffff;
	rx[6] = (data_cntr << 16) | temp_out;

	/* read TS, upper and lower are swapped in FPGA FIFO */
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &rx[9]);
	dev_dbg(&adis->spi->dev, "%s: got TS %08x\n", __func__, rx[9]);
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &rx[8]);
	dev_dbg(&adis->spi->dev, "%s: got TS %08x\n", __func__, rx[8]);

	/* frame delimiter 1 */
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &eop);
	dev_dbg(&adis->spi->dev, "%s: got eop %08x\n", __func__, eop);
	if (eop != adis->eop_1) {
		if (!atomic_read(&adis->started)) {
			dev_warn(&adis->spi->dev,
				"%s called though stopped\n", __func__);
			return;
		}
		dev_err(&adis->spi->dev,
			"%s: framing_1 error, got %08x, expected %08x\n",
			__func__, eop, adis->eop_1);
		dump_rx_buf(adis);
		imufifo_sync(adis);
		return;
	}
	/* frame delimiter 2 */
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_FIFO_OFFS, &eop);
	dev_dbg(&adis->spi->dev, "%s: got eop %08x\n", __func__, eop);
	if (eop != adis->eop_2) {
		dev_err(&adis->spi->dev, "%s: framing_2 error, got %08x\n",
				__func__, eop);
		imufifo_sync(adis);
		return;
	}

	ret = adis16465_insert_data(adis, adis->data32_rx_buffer,
			record_size);
	if (ret != record_size) {
		dev_err_ratelimited(&adis->spi->dev,
				"Failed to insert data: %d\n", ret);
	}
}

/* Only used for debugging, use register interface instead of FIFO */
static void __maybe_unused imufifo_read_regs(struct adis16465 *adis)
{
	int i;
	int ret;
	size_t record_size = sizeof(struct adis16465_data32_ts);
	uint16_t temp_out, data_cntr;
	uint32_t *rx;

	rx = adis->data32_rx_buffer;
	for (i = 1; i < 7; i++) {
		regmap_read(adis->mpf_regmap,
				MPF_PCTRL_ADIS_FIFO_OFFS + i * 4, rx);
		dev_dbg(&adis->spi->dev, "%s: got %08x\n",
				__func__, *rx);
		rx++;
	}
	/* temp_out is 16 bit, but FIFO datum is 32 bit */
	regmap_read(adis->mpf_regmap,
			MPF_PCTRL_ADIS_FIFO_OFFS + 7 * 4, rx);
	temp_out = *rx & 0xffff;
	regmap_read(adis->mpf_regmap,
			MPF_PCTRL_ADIS_FIFO_OFFS + 8 * 4, rx);
	/* data_cntr is 16 bit, but FIFO datum is 32 bit */
	data_cntr = *rx & 0xffff;
	*rx = (data_cntr << 16) | temp_out;
	rx++;	/* skip alignment for 64-bit TS */
	/* read TS */
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_TS_LO_OFFS, rx);
	rx++;
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_TS_HI_OFFS, rx);

	ret = adis16465_insert_data(adis, adis->data32_rx_buffer,
			record_size);
	if (ret != record_size) {
		dev_err(&adis->spi->dev,
				"Failed to insert data: %d\n", ret);
	}
}

/*
 * Data Ready IRQ handler
 */
irqreturn_t adis16465_dr_isr(int irq, void *p)
{
	struct adis16465 *adis = p;
	uint32_t status;
	int ret;

	if (testmode) {
		dr_ktime_timestamp = ktime_get();
		dr_isr_count++;
	}

	regmap_read(adis->mpf_regmap, MPF_PCTRL_STATUS_REG_OFFS, &status);
	if (!(status & MPF_PCTRL_STATUS_ADIS_INT)) {
		dev_warn(&adis->spi->dev, "%s not our IRQ, status %08x\n",
				__func__, status);
		return IRQ_NONE;
	}

	if (!imufifo && !atomic_read(&adis->started)) {
		dev_warn(&adis->spi->dev, "%s called though stopped\n",
			__func__);
		adis16465_dr_irq_disable(adis);
		adis16465_dr_irq_clr(adis);

		return IRQ_HANDLED;
	}

	/* Protect against a DR which is faster than our SPI msg. */
	adis16465_dr_irq_disable(adis);

	/* Check if we shall use FPGA implemented IMU data FIFO. */
	if (imufifo) {
		int count;

		if (status & MPF_PCTRL_STATUS_IMUFIFO_AEMPTY) {
			dev_warn(&adis->spi->dev,
				"%s IMUFIFO almost empty, status %08x\n",
				__func__, status);
			adis16465_dr_irq_clr(adis);
			adis16465_dr_irq_enable(adis);
			return IRQ_HANDLED;
		}

		if (status & MPF_PCTRL_STATUS_IMUFIFO_EMPTY) {
			dev_warn(&adis->spi->dev,
				"%s IMUFIFO empty, status %08x\n",
				__func__, status);
			adis16465_dr_irq_clr(adis);
			adis16465_dr_irq_enable(adis);
			return IRQ_HANDLED;
		}

		/*
		 * We cannot read until FIFO is empty, since we might get
		 * incomplete data sets.
		 * Instead we use the FPGA config constant.
		 */
		for (count = 0; count < IMUFIFO_DATA_SETS_PER_IRQ; count++)
			imufifo_read(adis);

		dev_dbg(&adis->spi->dev, "read %d sets\n", count);

		adis16465_dr_irq_clr(adis);
		if (atomic_read(&adis->started))
			adis16465_dr_irq_enable(adis);
		else
			complete(&adis->imufifo_completion);

		return IRQ_HANDLED;
	}

	adis16465_dr_irq_clr(adis);
	/*
	 * The TS is sync'ed with DR and is stable until next DR IRQ.
	 * NOTE: little endian CPU assumed.
	 */
#ifdef USE_KTIMESTAMP
	dr_timestamp = ktime_get_ns();
#else
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_TS_LO_OFFS,
						(uint32_t *)&dr_timestamp);
	regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_TS_HI_OFFS,
						(uint32_t *)&dr_timestamp + 1);
#endif
	if (adis->data_mode == DATA32)
		ret = spi_async(adis->spi, &adis->data32_msg);
	else
		ret = spi_async(adis->spi, &adis->burst_msg);

	if (ret)
		dev_err(&adis->spi->dev, "spi_async failed: ret %d\n", ret);

	return IRQ_HANDLED;
}

/*
 * Power-off IMU device via FPGA control register
 */
static void adis16465_poweroff(struct adis16465 *adis)
{
	int ret;
	struct device *dev = &adis->spi->dev;

	ret = regulator_disable(adis->regulator);
	if (ret)
		dev_err(dev, "Failed to disable adis16465 regulator\n");
}

/*
 * Power-on IMU device via FPGA control register
 */
static void adis16465_poweron(struct adis16465 *adis)
{
	int ret;
	struct device *dev = &adis->spi->dev;

	ret = regulator_enable(adis->regulator);
	if (ret)
		dev_err(dev, "Failed to enable adis16465 regulator\n");

	/* Wait Power-On Start-Up Time, before accessing registers */
	msleep(259);
}


static void adis16465_init_retry(struct adis16465 *adis)
{
	adis16465_poweroff(adis);
	adis16465_poweron(adis);
}

static int adis_init(struct adis16465 *adis)
{
	int rc = 0;
	struct device *dev = &adis->spi->dev;
	int retry_counter = 3;
	uint16_t reg;

	adis16465_poweron(adis);

	/*
	 * FPGA imufifo is enabled after reset for FPGA >= 0.6.5,
	 * switch it off so the driver can access the IMU register.
	 */
	if (imufifo)
		regmap_update_bits(adis->mpf_regmap, MPF_PCTRL_REG_OFFS,
				MPF_PCTRL_ADIS_SPI_SEL, 0);

	/*
	 * Workaround for HW/FPGA problems: very rarely the IMU is in an
	 * invalid state after poweron, try to detect it and power-cycle
	 * the IMU.
	 */
	while (retry_counter) {
		rc = adis_read_reg_16(adis, ADIS16465_REG_DIAG_STAT, &reg);
		if (rc) {
			dev_err(dev, "%s: Failed to get diag register: %d\n",
					__func__, rc);
			adis16465_init_retry(adis);
			--retry_counter;
			continue;
		}

		if (reg == 0)
			break;

		dev_warn(dev, "%s: diag register is 0x%x\n",
				__func__, reg);
		/*
		 * Check if it is cleared after reading.
		 * Reading this register causes all of its bits to return to 0.
		 * If an error condition persists, the flag (bit) automatically
		 * returns to an alarm value of 1.
		 */
		rc = adis_read_reg_16(adis, ADIS16465_REG_DIAG_STAT, &reg);
		if (!rc && reg) {
			dev_err(dev,
				"%s: diag register still set 0x%x\n",
				__func__, reg);
			adis16465_init_retry(adis);
		}

		--retry_counter;
		dev_info(dev, "%s: diag register: 0x%x, retries left %d\n",
				__func__, reg, retry_counter);
	}
	if (retry_counter == 0) {
		dev_err(dev, "IMU still in error state\n");
		rc = -ENXIO;
	}

	return rc;
}

/*
 * Character driver interface
 */

static inline struct adis16465 *file_to_adis_priv(struct file *file)
{
	struct miscdevice *dev = file->private_data;

	return container_of(dev, struct adis16465, miscdev);
}

static ssize_t adis16465_copy_to_user(struct adis16465 *adis, char __user *buf,
		size_t count)
{
	int rc;
	unsigned int copied;
	size_t copied_total = 0;
	size_t fifo_len;

	/*
	 * kfifo_to_user reads only one record, but we want to copy
	 * all available data, therefore loop over it.
	 */
	while (copied_total < count
			&& (fifo_len = kfifo_len(&adis->burst_fifo))) {
		rc = kfifo_to_user(&adis->burst_fifo, buf + copied_total,
				count - copied_total, &copied);
		dev_dbg(&adis->spi->dev,
				"%s, fifo_len %zu, count %zu, copied %u\n",
				__func__, fifo_len, count - copied_total,
				copied);
		if (rc) {
			dev_warn(&adis->spi->dev,
				 "Error during FIFO to userspace copy\n");
			return rc;
		}
		copied_total += copied;
	};

	return copied_total;
}

static ssize_t adis16465_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int rc;
	size_t copied;
	struct adis16465 *adis = file_to_adis_priv(file);

	dev_dbg(&adis->spi->dev, "%s, fifo_len %u, count %zd, started %d\n",
		__func__, kfifo_len(&adis->burst_fifo), count,
		atomic_read(&adis->started));

	copied = 0;
	do {
		/* block if no new data available */
		if (kfifo_is_empty(&adis->burst_fifo)) {
			if (file->f_flags & O_NONBLOCK) {
				if (copied)
					return copied;
				return -EAGAIN;
			}

			/* XXX: start trigger if not already started */

			dev_dbg(&adis->spi->dev, "%s waiting\n", __func__);
			rc = wait_event_interruptible(adis->burst_queue,
					!kfifo_is_empty(&adis->burst_fifo));
			dev_dbg(&adis->spi->dev, "%s woke\n", __func__);
			if (rc < 0)
				return -EINTR;
		}

		rc = adis16465_copy_to_user(adis, buf + copied, count - copied);
		if (rc < 0)
			return rc;

		copied += rc;

	} while (copied < count);

	return copied;
}

static unsigned int adis16465_poll(struct file *file,
					struct poll_table_struct *pt)
{
	struct adis16465 *adis = file_to_adis_priv(file);

	poll_wait(file, &adis->burst_queue, pt);
	if (!kfifo_is_empty(&adis->burst_fifo))
		return POLLIN | POLLRDNORM;

	return 0;
}

static int adis16465_open(struct inode *inode, struct file *file)
{
	struct adis16465 *adis = file_to_adis_priv(file);

	dev_dbg(&adis->spi->dev, "%s\n", __func__);

	if (test_and_set_bit(0, &adis->opened))
		return -EBUSY; /* already open */

	/* set default data mode */
	adis->data_mode = DATA32;
	adis->bursts_dropped = 0;
	adis->burst_fifo_high = 0;
	atomic_set(&adis->started, 0);

	if (testmode) {
		/* for debug purposes */
		last_data_cntr = 0;
	}

	return 0;
}

static int adis16465_release(struct inode *inode, struct file *file)
{
	struct adis16465 *adis = file_to_adis_priv(file);

	if (atomic_read(&adis->started))
		adis_ioctl_stop(adis, 0);

	clear_bit(0, &adis->opened);

	dev_dbg(&adis->spi->dev, "%s done\n", __func__);
	return 0;
}


static int adis_ioctl_reg_set(struct adis16465 *adis, unsigned long arg)
{
	struct adis_reg_set_get reg_addr_val;

	if (copy_from_user(&reg_addr_val, (const void __user *)arg,
				sizeof(reg_addr_val)))
		return -EFAULT;

	return adis_write_reg_16(adis, reg_addr_val.addr, reg_addr_val.val);
}

static int adis_ioctl_reg_get(struct adis16465 *adis, unsigned long arg)
{
	int rc;
	uint16_t val;
	void *to;
	const void *from;
	size_t size;
	struct adis_reg_set_get reg_addr_val;

	to = &reg_addr_val;
	from = (void *)arg;
	size = sizeof(struct adis_reg_set_get);
	if (copy_from_user(to, (const void __user *)from, size))
		return -EFAULT;

	rc = adis_read_reg_16(adis, reg_addr_val.addr, &val);
	if (rc)
		return rc;

	reg_addr_val.val = val;

	to = (void *)arg;
	from = &reg_addr_val;
	if (copy_to_user((void __user *)to, from, size))
		return -EFAULT;

	return rc;
}

static int adis_ioctl_data_mode_set(struct adis16465 *adis, unsigned long arg)
{
	int rc = 0;

	switch (arg) {
	case DATA32:
		adis->data_mode = DATA32;
		break;
	case BURST:
		adis->data_mode = BURST;
		break;
	case BURSTRAW:
		adis->data_mode = BURSTRAW;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int adis_ioctl_start(struct adis16465 *adis, unsigned long arg)
{
	int rc = 0;

	dev_dbg(&adis->spi->dev, "%s\n", __func__);

	if (atomic_read(&adis->started)) {
		dev_info(&adis->spi->dev, "%s: already started\n", __func__);
		return rc;
	}

	kfifo_reset(&adis->burst_fifo);

	/* Use FPGA internal SPI shift register */
	if (imufifo) {
		/* Make sure it is in the mode the FPGA expects. */
		adis16465_init_scaled_sync_mode(adis);

		reinit_completion(&adis->imufifo_completion);
		/* Get current end-of-packet/frame seperator. */
		regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_SEPWORD1_OFFS,
				&adis->eop_1);
		regmap_read(adis->mpf_regmap, MPF_PCTRL_ADIS_SEPWORD2_OFFS,
				&adis->eop_2);
		regmap_update_bits(adis->mpf_regmap, MPF_PCTRL_REG_OFFS,
				MPF_PCTRL_ADIS_SPI_SEL, MPF_PCTRL_ADIS_SPI_SEL);
	}
	atomic_set(&adis->started, 1);

	/* Polling mode */
	if (!adis->irq)
		hrtimer_start(&adis->timer, adis->period, HRTIMER_MODE_REL);
	else
		adis16465_dr_irq_enable(adis);

	return rc;
}

static int adis_ioctl_stop(struct adis16465 *adis, unsigned long arg)
{
	int rc = 0;

	/* Do nothing if already stopped. */
	if (!atomic_read(&adis->started)) {
		dev_dbg(&adis->spi->dev, "%s: already stopped\n", __func__);
		return rc;
	}

	atomic_set(&adis->started, 0);

	if (!imufifo) {
		adis16465_dr_irq_disable(adis);
		adis16465_dr_irq_clr(adis);
	}

	dev_dbg(&adis->spi->dev, "%s\n", __func__);

	/* Polling mode */
	if (!adis->irq)
		hrtimer_cancel(&adis->timer);

	cancel_work_sync(&adis->irq_work);

	kfifo_reset(&adis->burst_fifo);

	if (imufifo) {
		/*
		 * Wait for IRQ handler completion, otherwise the handler might
		 * issue framing errors if we switch off the imufifo mode.
		 */
		if (!wait_for_completion_timeout(&adis->imufifo_completion,
					msecs_to_jiffies(100))) {
			dev_err(&adis->spi->dev,
					"%s wait_for_completion timed out\n",
					__func__);
			rc = -ETIMEDOUT;
		};

		/*
		 * Disable FPGA internal SPI shift register and make SPI
		 * available for the driver.
		 */
		regmap_update_bits(adis->mpf_regmap, MPF_PCTRL_REG_OFFS,
			MPF_PCTRL_ADIS_SPI_SEL, 0);

		/*
		 * XXX: FPGA work-around. The application must do its IMU
		 * settings again after stopping and before start.
		 * IMU might got some SPI signal from FPGA IMU SPI after
		 * stopping it. Get it into a clean state.
		 */
		adis16465_poweroff(adis);
		adis_init(adis);
		adis16465_init_scaled_sync_mode(adis);

		dev_dbg(&adis->spi->dev, "%s completed\n", __func__);
	}

	return rc;
}

static int adis_ioctl_sync_mode_get(struct adis16465 *adis,
		unsigned long arg)
{
	enum adis_sync_mode mode;
	int rc;

	dev_dbg(&adis->spi->dev, "%s\n", __func__);


	rc = adis16465_sync_mode_get(adis, &mode);
	if (rc)
		return rc;

	if (copy_to_user((void __user *)arg, &mode, sizeof(mode)))
		return -EFAULT;

	return 0;
}

static int adis_ioctl_sync_mode_set(struct adis16465 *adis,
		unsigned long arg)
{
	enum adis_sync_mode mode = arg;
	uint16_t val;
	int rc;

	dev_dbg(&adis->spi->dev, "%s\n", __func__);

	if (copy_from_user(&mode, (const void __user *)arg, sizeof(mode)))
		return -EFAULT;

	if (mode == OUT_SYNC) {
		/* two outputs would collide */
		dev_err(&adis->spi->dev, "forbidden mode %d\n", mode);
		return -EINVAL;
	}

	rc = adis_read_reg_16(adis, ADIS16465_REG_MSC_CTRL, &val);
	if (rc)
		return rc;

	val &= ~(MSC_CTRL_SYNC_MASK << MSC_CTRL_SYNC_SHIFT);
	val |= (mode << MSC_CTRL_SYNC_SHIFT);

	rc = adis_write_reg_16(adis, ADIS16465_REG_MSC_CTRL, val);
	if (rc)
		return rc;

	return 0;
}

static long adis16465_ioctl(struct file *file, unsigned int ioc,
				unsigned long arg)
{
	int rc;
	struct adis16465 *adis = file_to_adis_priv(file);

	dev_dbg(&adis->spi->dev, "%s\n", __func__);

	switch (ioc) {
	case ADIS_IOC_REG_SET:
		rc = adis_ioctl_reg_set(adis, arg);
		break;
	case ADIS_IOC_REG_GET:
		rc = adis_ioctl_reg_get(adis, arg);
		break;
	case ADIS_IOC_DATA_MODE_SET:
		rc = adis_ioctl_data_mode_set(adis, arg);
		break;
	case ADIS_IOC_START:
		rc = adis_ioctl_start(adis, arg);
		break;
	case ADIS_IOC_STOP:
		rc = adis_ioctl_stop(adis, arg);
		break;
	case ADIS_IOC_SYNC_MODE_GET:
		rc = adis_ioctl_sync_mode_get(adis, arg);
		break;
	case ADIS_IOC_SYNC_MODE_SET:
		rc = adis_ioctl_sync_mode_set(adis, arg);
		break;
	case ADIS_IOC_SAMPLE_RATE_GET:
		rc = adis16465_get_freq(adis);
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

const struct file_operations adis16465_fops = {
	.owner            = THIS_MODULE,
	.read             = adis16465_read,
	.poll		  = adis16465_poll,
	.unlocked_ioctl   = adis16465_ioctl,
	.open             = adis16465_open,
	.release          = adis16465_release,
};

static int adis16465_init_chrdev(struct adis16465 *adis)
{
	int rc;

	adis->miscdev.minor = MISC_DYNAMIC_MINOR;
	adis->miscdev.name = DRIVER_NAME;
	adis->miscdev.fops = &adis16465_fops;

	rc = misc_register(&adis->miscdev);
	if (rc < 0) {
		pr_err("%s: misc_register returns %d.\n",
				DRIVER_NAME, rc);
		return rc;
	}
	return 0;
}

static int adis16465_probe(struct spi_device *spi)
{
	struct adis16465 *adis;
	int ret;
	int irq;
	struct regmap *regmap;

	adis = devm_kzalloc(&spi->dev, sizeof(*adis), GFP_KERNEL);
	if (!adis)
		return -ENOMEM;

	adis->spi = spi;
	spi_set_drvdata(spi, adis);

	dev_info(&spi->dev, "%s\n", __func__);

	/*
	 * Because the DR IRQ enable and acknowledge is in a register which is
	 * shared with other drivers, we need to access it via the
	 * corresponding regmap, which is created in mpf.ko.
	 * mpf.ko -> mpf_spi.ko -> spi or by dev_name:
	 * "0001:01:00.0" -> "mpf-spi.42" -> "spi" -> "adis16465"
	 * So we can either use "spi->dev.parent->parent->parent"
	 * or d = bus_find_device_by_name(&pci_bus_type, NULL, "0001:01:00.0")
	 * Both are not elegant, but are avoiding external references.
	 */
	regmap = dev_get_regmap(spi->dev.parent->parent->parent, NULL);
	if (!regmap) {
		dev_err(&spi->dev, "Cannot get regmap.\n");
		return -ENODEV;
	}
	adis->mpf_regmap = regmap;

	mutex_init(&adis->txrx_lock);

	adis->regulator = devm_regulator_get(&spi->dev, "V-adis");
	if (IS_ERR(adis->regulator)) {
		dev_err(&spi->dev, "Failed to get adis16465 regulator\n");
		ret = PTR_ERR(adis->regulator);
		adis->regulator = NULL;
		return ret;
	}

	ret = adis_init(adis);
	if (ret)
		return ret;

	irq = spi->irq;
	ret = devm_request_threaded_irq(&spi->dev, irq, NULL,
						adis16465_dr_isr, 0,
						DRIVER_NAME, adis);
	if (ret) {
		dev_err(&spi->dev, "%s: Can't allocate irq %d\n", __func__,
						irq);
	} else
		adis->irq = irq;

	adis16465_debugfs_init(adis);

	adis16465_init_chrdev(adis);

	init_waitqueue_head(&adis->burst_queue);
	ret = kfifo_alloc(&adis->burst_fifo, ADIS_BURST_FIFO_SIZE, GFP_KERNEL);
	if (ret)
		dev_err(&spi->dev, "kfifo_alloc failed: %d\n", ret);

	INIT_WORK(&adis->irq_work, adis16465_irq_work);
	if (!adis->irq) {
		dev_info(&spi->dev,
			"No IRQ number specified, will use polling\n");
		/*
		 * polling mode with hrtimer
		 */
		hrtimer_init(&adis->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		adis->timer.function = adis16465_hrtimer_trig_handler;
		adis->sampling_frequency = 200;
		adis->period =
			ktime_set(0, NSEC_PER_SEC / adis->sampling_frequency);
	}

	adis->f_sync = MPF_ADIS_SYNC_FREQ; /* hard-coded in the FPGA */

	adis16465_prepare_burst_msg(adis);
	adis16465_prepare_data32_msg(adis);

	adis16465_init_scaled_sync_mode(adis);

	init_completion(&adis->imufifo_completion);

	/*
	 * XXX: FPGA/HW powerup workaround, get into clean state
	 * and throw away erroneous data.
	 */
	if (imufifo) {
		adis_ioctl_start(adis, 0);
		adis_ioctl_stop(adis, 0);
	}

	dev_info(&spi->dev, "%s: scaled sync @ 200 Hz\n", __func__);
	return 0;
}

static int adis16465_remove(struct spi_device *spi)
{
	struct adis16465 *adis = spi_get_drvdata(spi);

	dev_info(&spi->dev, "%s\n", __func__);

	if (!adis->irq)
		hrtimer_cancel(&adis->timer);
	cancel_work_sync(&adis->irq_work);

	kfifo_free(&adis->burst_fifo);

	/* does nothing if parameter is NULL */
	debugfs_remove_recursive(adis->debugfs_dentry);

	kfree(adis->burst_xfer);
	kfree(adis->burst_cmd_buffer);
	kfree(adis->data32_xfer);
	kfree(adis->data32_buffer);

	misc_deregister(&adis->miscdev);

	adis16465_poweroff(adis);

	return 0;
}

static const struct spi_device_id adis16465_ids[] = {
	{ "adis16465", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, adis16465_ids);

static struct spi_driver adis16465_driver = {
	.driver = {
		.name = "adis16465",
	},
	.id_table = adis16465_ids,
	.probe = adis16465_probe,
	.remove = adis16465_remove,
};
module_spi_driver(adis16465_driver);

MODULE_AUTHOR("Michael Brandt <michael.brandt@devtwig.se>");
MODULE_DESCRIPTION("Analog Devices ADIS16465 IMU driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);


// SPDX-License-Identifier: GPL-2.0

//#define DEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/circ_buf.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/ktime.h>

#include "mpf_uart.h"

#define DRV_NAME "mpf_uart"
#define DRV_VERSION "0.9"

#define FPGA_CLK 125000000

#define UART_BAUDRATE_OFFS 0x00
#define UART_FRACT_BAUDRATE_OFFS 0x04
#define UART_PARAMS_OFFS 0x08
#define UART_MAX_SIZE_OFFS 0x0c
#define UART_STATUS_OFFS 0x10
#define UART_ERROR_OFFS 0x14
#define UART_ERROR_TX_OVERFLOW (1 << 0)
#define UART_ERROR_TX_UNDERFLOW (1 << 1)
#define UART_ERROR_RX_OVERFLOW (1 << 4)
#define UART_ERROR_RX_UNDERFLOW (1 << 5)
#define UART_ERROR_RX_FIFO_FULL (1 << 6)
#define UART_ERROR_UART_FRAMING (1 << 8)
#define UART_ERROR_UART_OVERFLOW (1 << 9)
#define UART_ERROR_UART_PARITY (1 << 10)
#define UART_ERRORCOUNT_OFFS 0x18
#define UART_RX_IDLE_TIMEOUT_OFFS 0x1c
#define UART_HOLD_OFF_TIME_OFFS 0x20
#define UART_AMT_OF_BYTES_AVAIL_TX_OFFS 0x24
#define UART_AMT_OF_BYTES_AVAIL_RX_OFFS 0x28
#define UART_SPARE_DATA_TX_OFFS 0x2c
#define UART_SPARE_DATA_TX_BE_SHIFT 24
#define UART_SPARE_DATA_RX_OFFS 0x30
#define UART_SPARE_DATA_RX_BE_SHIFT 24
#define UART_DATA_TX_OFFS 0x1000
#define UART_DATA_RX_OFFS 0x2000

static unsigned int buffer_size = 4096;
module_param(buffer_size, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(buffer_size, "Size of the FIFO buffers");

static unsigned int write_timeout_ms = 0;
module_param(write_timeout_ms, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(write_timeout_ms, "Write timeout in ms or 0 if deactivated");

static unsigned int read_timeout_ms = 0;
module_param(read_timeout_ms, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(read_timeout_ms, "Read timeout in ms or 0 if deactivated");

static unsigned int rx_poll_time_ms = 1;
module_param(rx_poll_time_ms, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(rx_poll_time_ms, "Rx poll time in ms or 0 if deactivated");

static unsigned int rx_idle_timeout = 0;
module_param(rx_idle_timeout, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(rx_idle_timeout, "Receive FIFO idle timeout (in FPGA ticks or 0 if deactivated)");

static unsigned int hold_off_time = 8000*125; // 8 msec
module_param(hold_off_time, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(hold_off_time, "Min. time between two interrupts (in FPGA ticks)");

/*
 * The baudrate is configured by baud_val and baud_val_fraction. Refer to the
 * Microsemi CoreUART manual. The system clock is 125 MHz.
 *
 * desired baudrate  baud_val  fract.  actual baudrate
 * ----------------  --------  ------  ---------------
 *             9600       812       6           9600.6
 *            19200       405       7          19201.2
 *            38400       202       4          38390.7
 *           115200        66       7         115101.3
 *          1000000         6       7         992063.5
 *          1152000         5       6        1157407.4
 *          1500000         4       2        1488095.2
 *          2000000         2       7        2016129.0
 *          2500000         2       1        2500000.0
 *          3000000         1       5        2976190.5
 */

static unsigned int baud_val = 6; // 6;
module_param(baud_val, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(baud_val, "BAUD_VAL register setting");

static unsigned int baud_val_fraction = 7;
module_param(baud_val_fraction, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(baud_val_fraction, "BAUD_VAL_FRACTION register setting");

static unsigned int uart_params = 1; // 8N1
module_param(uart_params, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(uart_params, "UART paramters (data width and parity)");

static struct class *mpf_class;	/* from parent */

struct mpf_buf {
	char *buf;
	atomic_t head;
	atomic_t tail;
	int size;
	wait_queue_head_t pushed;
	wait_queue_head_t avail_to_push;
};

static struct mpf_buf *mpf_buf_alloc(int size)
{
	struct mpf_buf *buf;

	if (size <= 0)
		return NULL;
	if (size & (size - 1))
		return NULL; // is not power of 2

	buf = kzalloc(sizeof(struct mpf_buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	buf->buf = kzalloc(size, GFP_KERNEL);
	if (!buf->buf) {
		kfree(buf);
		return NULL;
	}

	buf->size = size;
	init_waitqueue_head(&buf->pushed);
	init_waitqueue_head(&buf->avail_to_push);

	return buf;
}

static void mpf_buf_free(struct mpf_buf *buf)
{
	kfree(buf->buf);
	kfree(buf);
}

struct mpf_uart {
	struct list_head list;
	dev_t dev;
	struct cdev cdev;
	struct device *device;
	struct mpf_buf *rx;
	struct mpf_buf *tx;
	void __iomem *base;
	uint32_t fifo_size;
	struct task_struct *rx_task;
	struct task_struct *tx_task;
	spinlock_t lock;
	struct file *read;
	struct file *write;
};

static struct {
	struct list_head uart_list;
	struct class *class;
	struct mpf_buf *x, *y, *z;
} private = {};

static int mpf_uart_open(struct inode *inode, struct file *filp)
{
	struct mpf_uart *uart =
		container_of(inode->i_cdev, struct mpf_uart, cdev);
	int mode = (filp->f_flags & O_ACCMODE);
	int read = 0, write = 0;
	unsigned long flags;

	dev_dbg(uart->device, "open %#x\n", filp->f_flags);

	if (mode == O_RDONLY || mode == O_RDWR)
		read = 1;
	if (mode == O_WRONLY || mode == O_RDWR)
		write = 1;
	if (read && uart->read != NULL)
		return -EBUSY;
	if (write && uart->write != NULL)
		return -EBUSY;

	spin_lock_irqsave(&uart->lock, flags);
	if (uart->base != NULL) {
		if (uart->rx_task == NULL || uart->tx_task == NULL) {
			spin_unlock_irqrestore(&uart->lock, flags);
			return -EAGAIN;
		}
	}
	if (read)
		uart->read = filp;
	if (write)
		uart->write = filp;
	spin_unlock_irqrestore(&uart->lock, flags);

	filp->private_data = uart;

	return 0;
}

static int mpf_uart_release(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct mpf_uart *uart =
		container_of(inode->i_cdev, struct mpf_uart, cdev);

	dev_dbg(uart->device, "release\n");

	spin_lock_irqsave(&uart->lock, flags);
	if (uart->write == filp)
		uart->write = NULL;
	if (uart->read == filp)
		uart->read = NULL;
	spin_unlock_irqrestore(&uart->lock, flags);

	return 0;
}

static ssize_t _mpf_uart_read(struct file *filp, char *buffer, size_t len,
	loff_t *offset)
{
	struct mpf_uart *uart = filp->private_data;
	int head = -1, tail, count, rest, ret;

	dev_dbg(uart->device, "read\n");

	if (uart->rx == NULL)
		return -EFAULT;
	if (len <= 0)
		return 0;

	while (1) {
		if (read_timeout_ms > 0) {
			ret = wait_event_interruptible_timeout(uart->rx->pushed,
				head != atomic_read(&uart->rx->head),
				msecs_to_jiffies(read_timeout_ms));
			if (ret == 0)
				return -ETIMEDOUT;
			if (ret == -ERESTARTSYS)
				return -EINTR;
			if (ret < 0)
				return ret;
		} else {
			ret = wait_event_interruptible(uart->rx->pushed,
				head != atomic_read(&uart->rx->head));
			if (ret == -ERESTARTSYS)
				return -EINTR;
			if (ret < 0)
				return ret;
		}
		head = atomic_read(&uart->rx->head);
		tail = atomic_read(&uart->rx->tail);
		count = CIRC_CNT_TO_END(head, tail, uart->rx->size);
		if (count > 0)
			break;
	}

	if (len < count)
		count = len;
	if (copy_to_user(buffer, &uart->rx->buf[tail], count))
		return -EFAULT;
	len -= count;
	tail = (tail + count) & (uart->rx->size - 1);
	rest = CIRC_CNT_TO_END(head, tail, uart->rx->size);
	if (len > 0 && rest > 0)
	{
		if (len < rest)
			rest = len;
		if (copy_to_user(&buffer[count], &uart->rx->buf[tail], rest))
			return -EFAULT;
		count += rest;
		tail = (tail + rest) & (uart->rx->size - 1);
	}
	atomic_set(&uart->rx->tail, tail);
	dev_dbg(uart->device, "read count = %d\n", count);

	return count;
}

static ssize_t mpf_uart_read(struct file *filp, char *buffer, size_t len,
	loff_t *offset)
{
	ssize_t rc;
	ktime_t start_time, stop_time;

	start_time = ktime_get();
	rc = _mpf_uart_read(filp, buffer, len, offset);
	stop_time = ktime_get();
	pr_debug("%s: read time %llu\n", __func__,
			ktime_to_us(ktime_sub(stop_time, start_time)));
	return rc;
}

static ssize_t _mpf_uart_write(struct file *filp, const char *buffer, size_t len,
	loff_t *offset)
{
	struct mpf_uart *uart = filp->private_data;
	int head, tail, count, rest;
	unsigned int timeout_ms = write_timeout_ms;
	unsigned long timeout = jiffies + msecs_to_jiffies(timeout_ms);

	dev_dbg(uart->device, "write\n");

	if (uart->tx == NULL)
		return -EFAULT;
	if (len <= 0)
		return 0;

	head = atomic_read(&uart->tx->head);
	while (1) {
		tail = atomic_read(&uart->tx->tail);
		count = CIRC_SPACE_TO_END(head, tail, uart->tx->size);
		if (count > 0)
			break;
		if (timeout_ms > 0)
			if (time_after(jiffies, timeout))
				return -ETIMEDOUT;
		msleep(20);
	}

	if (len < count)
		count = len;
	if (copy_from_user(&uart->tx->buf[head], buffer, count))
		return -EFAULT;
	len -= count;
	head = (head + count) & (uart->tx->size - 1);
	rest = CIRC_SPACE_TO_END(head, tail, uart->tx->size);
	if (len > 0 && rest > 0)
	{
		if (len < rest)
			rest = len;
		if (copy_from_user(&uart->tx->buf[head], &buffer[count], rest))
			return -EFAULT;
		count += rest;
		head = (head + rest) & (uart->tx->size - 1);
	}
	atomic_set(&uart->tx->head, head);
	dev_dbg(uart->device, "write count = %d\n", count);
	wake_up_interruptible(&uart->tx->pushed);

	return count;
}

static ssize_t mpf_uart_write(struct file *filp, const char *buffer, size_t len,
	loff_t *offset)
{
	ssize_t rc;
	ktime_t start_time, stop_time;

	start_time = ktime_get();
	rc = _mpf_uart_write(filp, buffer, len, offset);
	stop_time = ktime_get();
	pr_debug("write time %llu\n",
			ktime_to_us(ktime_sub(stop_time, start_time)));
	return rc;
}

static unsigned int mpf_uart_poll(struct file *filp, poll_table *wait)
{
	struct mpf_uart *uart = filp->private_data;
	int head, tail, count;
	unsigned int mask = 0;
	static ktime_t start_time, stop_time;

	dev_dbg(uart->device, "poll\n");

	poll_wait(filp, &uart->rx->pushed, wait);

	head = atomic_read(&uart->tx->head);
	tail = atomic_read(&uart->tx->tail);
	count = CIRC_SPACE(head, tail, uart->tx->size);
	if (count > 0)
		mask |= POLLOUT | POLLWRNORM;

	head = atomic_read(&uart->rx->head);
	tail = atomic_read(&uart->rx->tail);
	count = CIRC_CNT(head, tail, uart->rx->size);
	if (count > 0) {
		stop_time = ktime_get();
		mask |= POLLIN | POLLRDNORM;
		dev_dbg(uart->device, "%s: poll rd time %llu\n", __func__,
				ktime_to_us(ktime_sub(stop_time, start_time)));
	} else
		start_time = ktime_get();

	return mask;
}

#define ior32(addr) ior32_(__func__, addr)
#define iow32(val, addr) iow32_(__func__, val, addr)

static unsigned int ior32_(const char *func, void __iomem *addr)
{
	unsigned int value = ioread32(addr);
//	pr_debug("{%s} read  %p => %#x\n", func, addr, value);
	return value;
}

static void iow32_(const char *func, u32 value, void __iomem *addr)
{
//	pr_debug("{%s} write %p <= %#x\n", func, addr, value);
	iowrite32(value, addr);
}

static long mpf_uart_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mpf_uart *uart = filp->private_data;
	struct mpf_uart_status status = {};
	void __user *buffer = (void __user *)arg;

	dev_dbg(uart->device, "ioctl %#x %#lx\n", cmd, arg);

	switch (cmd) {
		case MPF_UART_STATUS:
			if (uart->base != NULL) {
				status.error = ior32(uart->base + UART_ERROR_OFFS);
				status.error_count = ior32(uart->base + UART_ERRORCOUNT_OFFS);
			}
			if (copy_to_user(buffer, &status, sizeof(status)))
				return -EFAULT;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static irqreturn_t mpf_uart_isr(int irq, void *arg)
{
	struct platform_device *pdev = arg;
	struct mpf_uart *uart = platform_get_drvdata(pdev);
	pr_debug("%s\n", __func__);
	wake_up_interruptible(&uart->rx->avail_to_push);
	return IRQ_HANDLED;
}

static void mpf_uart_rx(struct mpf_uart *uart)
{
	int head, tail, count;
	uint32_t available_to_read;
	while (!kthread_should_stop()) {
		head = atomic_read(&uart->rx->head);
		tail = atomic_read(&uart->rx->tail);
		count = CIRC_SPACE(head, tail, uart->rx->size);
		if (count < 4)
			break; // write data to userspace first
//		available_to_read = ior32(uart->base + UART_AMT_OF_BYTES_AVAIL_RX_OFFS);
		available_to_read = ioread32(uart->base + UART_AMT_OF_BYTES_AVAIL_RX_OFFS);
		dev_dbg(uart->device, "available_to_read = %u (count = %d)\n", available_to_read, count);
		if (available_to_read == 0) {
			return;
		}
		if ((available_to_read + 3) > count) {
			// Reserve space in the buffer. In case some bytes are received
			// while reading, SPARE_DATA_RX will read up to 3 bytes.
			available_to_read = count - 3;
		}
		dev_dbg(uart->device, "receiving %u bytes\n", available_to_read);
		while (available_to_read >= 4) {
			uint32_t data = ior32(uart->base + UART_DATA_RX_OFFS);
			uart->rx->buf[head] = (data & 0xff);
			head = (head + 1) & (uart->rx->size - 1);
			uart->rx->buf[head] = ((data >> 8) & 0xff);
			head = (head + 1) & (uart->rx->size - 1);
			uart->rx->buf[head] = ((data >> 16) & 0xff);
			head = (head + 1) & (uart->rx->size - 1);
			uart->rx->buf[head] = ((data >> 24) & 0xff);
			head = (head + 1) & (uart->rx->size - 1);
			available_to_read -= 4;
		}
		if (available_to_read > 0) {
			uint32_t data = ior32(uart->base + UART_SPARE_DATA_RX_OFFS);
			int byte_enable = ((data >> UART_SPARE_DATA_RX_BE_SHIFT) & 3) + 1;
			if (available_to_read > byte_enable) {
				dev_dbg(uart->device, "something's wrong %u %d\n", available_to_read, byte_enable);
			}
			while (byte_enable > 0) {
				uart->rx->buf[head] = (data & 0xff);
				head = (head + 1) & (uart->rx->size - 1);
				data >>= 8;
				byte_enable--;
			}
		}
		dev_dbg(uart->device, "done receiving (%u)!\n", available_to_read);
		atomic_set(&uart->rx->head, head);
		break;
	}
	wake_up_interruptible(&uart->rx->pushed);
}

static void mpf_uart_tx(struct mpf_uart *uart)
{
	int head, tail, available_to_write;
	uint32_t free_space;
	tail = atomic_read(&uart->tx->tail);
	while (!kthread_should_stop()) {
		head = atomic_read(&uart->tx->head);
		available_to_write = CIRC_CNT_TO_END(head, tail, uart->tx->size);
		if (available_to_write == 0) {
			// everything was written to the FPGA
			atomic_set(&uart->tx->tail, tail);
			break;
		}
		free_space = ior32(uart->base + UART_AMT_OF_BYTES_AVAIL_TX_OFFS);
		if (free_space == 0) {
			// wait until FPGA empties the buffer
			usleep_range(500, 1500);
			continue;
		}
		if (available_to_write > free_space)
			available_to_write = free_space;

		dev_dbg(uart->device, "writing %u bytes\n", available_to_write);

		while (available_to_write >= 4) {
			uint32_t data = uart->tx->buf[tail];
			tail = (tail + 1) & (uart->tx->size - 1);
			data |= uart->tx->buf[tail] << 8;
			tail = (tail + 1) & (uart->tx->size - 1);
			data |= uart->tx->buf[tail] << 16;
			tail = (tail + 1) & (uart->tx->size - 1);
			data |= uart->tx->buf[tail] << 24;
			tail = (tail + 1) & (uart->tx->size - 1);
			iow32(data, uart->base + UART_DATA_TX_OFFS);
			available_to_write -= 4;
		}
		while (available_to_write > 0) {
			uint8_t byte_enable = 0;
			uint32_t data = uart->tx->buf[tail];
			tail = (tail + 1) & (uart->tx->size - 1);
			available_to_write--;
			if (available_to_write > 0) {
				data |= uart->tx->buf[tail] << 8;
				tail = (tail + 1) & (uart->tx->size - 1);
				available_to_write--;
				byte_enable++;
			}
			if (available_to_write > 0) {
				data |= uart->tx->buf[tail] << 16;
				tail = (tail + 1) & (uart->tx->size - 1);
				available_to_write--;
				byte_enable++;
			}
			data = ((byte_enable & 3) << UART_SPARE_DATA_TX_BE_SHIFT) | (data & 0x00ffffff);
			iow32(data, uart->base + UART_SPARE_DATA_TX_OFFS);
		}
		dev_dbg(uart->device, "done writing (%u)\n", available_to_write);
	}
}

static void mpf_uart_check_error(struct mpf_uart *uart)
{
	uint32_t error_code, error_count, e;

//	error_code = ior32(uart->base + UART_ERROR_OFFS);
	error_code = ioread32(uart->base + UART_ERROR_OFFS);
	if (error_code == 0)
		return;

	e = error_code;
	if (e & UART_ERROR_TX_OVERFLOW) {
		dev_err(uart->device, "TX overflow\n");
		e &= ~UART_ERROR_TX_OVERFLOW;
	}
	if (e & UART_ERROR_TX_UNDERFLOW) {
		dev_err(uart->device, "TX underflow\n");
		e &= ~UART_ERROR_TX_UNDERFLOW;
	}
	if (e & UART_ERROR_RX_OVERFLOW) {
		dev_err(uart->device, "RX overflow\n");
		e &= ~UART_ERROR_RX_OVERFLOW;
	}
	if (e & UART_ERROR_RX_UNDERFLOW) {
		dev_err(uart->device, "RX underflow\n");
		e &= ~UART_ERROR_RX_UNDERFLOW;
	}
	if (e & UART_ERROR_RX_FIFO_FULL) {
		dev_err(uart->device, "RX FIFO full\n");
		e &= ~UART_ERROR_RX_FIFO_FULL;
	}
	if (e & UART_ERROR_UART_FRAMING) {
		dev_err(uart->device, "UART framing error\n");
		e &= ~UART_ERROR_UART_FRAMING;
	}
	if (e & UART_ERROR_UART_OVERFLOW) {
		dev_err(uart->device, "UART overflow\n");
		e &= ~UART_ERROR_UART_OVERFLOW;
	}
	if (e & UART_ERROR_UART_PARITY) {
		dev_err(uart->device, "UART parity error\n");
		e &= ~UART_ERROR_UART_PARITY;
	}
	if (e & 0x80) {
		e &= ~0x80; // ignore this error bit
	}

	// Remove FPGA debugging flags
	e &= 0xfff;

	if (e != 0) {
		dev_err(uart->device, "unknown error state: %#x\n", error_code);
	}

//	error_count = ior32(uart->base + UART_ERRORCOUNT_OFFS);
	error_count = ioread32(uart->base + UART_ERRORCOUNT_OFFS);
	if (error_count != 0 && e != 0) {
		dev_err(uart->device, "error = %#x, count = %u\n", error_code, error_count);
	}

	// reset errors
//	iow32(0, uart->base + UART_ERROR_OFFS);
	iowrite32(0, uart->base + UART_ERROR_OFFS);
//	iow32(0, uart->base + UART_ERRORCOUNT_OFFS);
	iowrite32(0, uart->base + UART_ERRORCOUNT_OFFS);
}

static int mpf_uart_rx_thread(void *data)
{
	struct mpf_uart *uart = data;
	int ret;
	ktime_t start_time, stop_time;
	unsigned int timeout_counter;

	if (uart == NULL || uart->base == NULL) {
		pr_err("%s: null pointer\n", __func__);
		return -ENXIO;
	}

	pr_debug("%s: started\n", __func__);
	while (!kthread_should_stop()) {
		start_time = ktime_get();
		timeout_counter = rx_poll_time_ms * 1000 / 50;
		while (timeout_counter-- && !kthread_should_stop()) {
			ret = ioread32(uart->base
					+ UART_AMT_OF_BYTES_AVAIL_RX_OFFS);
			if (ret)
				break;
			usleep_range(50, 150);
		}
		stop_time = ktime_get();
		pr_debug("%s: ioread32 %llu\n", __func__,
				ktime_to_us(ktime_sub(stop_time, start_time)));
		if (ret == 0)
			continue;

		if (kthread_should_stop())
			break;

		mpf_uart_rx(uart);
		mpf_uart_check_error(uart);
	}
	pr_debug("%s: stopped\n", __func__);
	return 0;
}

static int mpf_uart_tx_thread(void *data)
{
	struct mpf_uart *uart = data;
	int head = -1, ret;

	if (uart == NULL || uart->base == NULL) {
		pr_err("%s: null pointer\n", __func__);
		return -ENXIO;
	}

	pr_debug("%s: started\n", __func__);
	while (!kthread_should_stop()) {
		ret = wait_event_interruptible(uart->tx->pushed,
			kthread_should_stop() || head != atomic_read(&uart->tx->head));

		if (ret == -ERESTARTSYS)
			continue;
		if (ret < 0) {
			pr_err("%s: wait_event_interruptible failed = %d\n", __func__, ret);
			return ret;
		}

		if (kthread_should_stop())
			break;

		mpf_uart_tx(uart);
		head = atomic_read(&uart->tx->head);
		mpf_uart_check_error(uart);
	}
	pr_debug("%s: stopped\n", __func__);
	return 0;
}

static const struct file_operations mpf_uart_fops = {
	.owner          = THIS_MODULE,
	.open           = mpf_uart_open,
	.release	= mpf_uart_release,
	.read           = mpf_uart_read,
	.write          = mpf_uart_write,
	.poll		= mpf_uart_poll,
	.unlocked_ioctl	= mpf_uart_ioctl,
};

static struct mpf_uart *mpf_uart_create(const char *name, struct mpf_buf *rx,
	struct mpf_buf *tx, void __iomem *base)
{
	int ret;
	struct mpf_uart *uart;
	struct task_struct *rx_task, *tx_task;
	uint32_t fifo_size = 0;

	if (base != NULL) {
		fifo_size = ior32(base + UART_MAX_SIZE_OFFS);
		if (fifo_size <= 4) {
			pr_err("invalid FIFO size: %u\n", fifo_size);
			return ERR_PTR(-ENOMEM);
		}
	}

	pr_debug("creating cdev: %s\n", name);

	uart = kzalloc(sizeof(struct mpf_uart), GFP_KERNEL);
	if (!uart)
		return ERR_PTR(-ENOMEM);

	uart->rx = rx;
	uart->tx = tx;
	uart->base = base;
	uart->fifo_size = fifo_size;

	ret = alloc_chrdev_region(&uart->dev, 0, 1, name);
	if (ret) {
		pr_err("failed to allocate chrdev region: %d\n", ret);
		goto alloc_chrdev_region_failed;
	}

	cdev_init(&uart->cdev, &mpf_uart_fops);
	uart->cdev.owner = mpf_uart_fops.owner;
	ret = cdev_add(&uart->cdev, uart->dev, 1);
	if (ret) {
		pr_err("failed to add cdev\n");
		goto cdev_add_failed;
	}

	uart->device = device_create(private.class, NULL, uart->dev, NULL,
		name);

	if (!uart->device) {
		pr_err("failed to create device\n");
		ret = -ENODEV;
		goto device_create_failed;
	}
	dev_dbg(uart->device, "device created\n");

	if (base != NULL) {
		unsigned long flags;

		rx_task = kthread_run(mpf_uart_rx_thread, uart, "%s_rx", name);
		if (IS_ERR(rx_task)) {
			ret = PTR_ERR(rx_task);
			pr_err("failed to create rx thread: %d\n", ret);
			goto kthread_run_rx_failed;
		}

		tx_task = kthread_run(mpf_uart_tx_thread, uart, "%s_tx", name);
		if (IS_ERR(tx_task)) {
			ret = PTR_ERR(tx_task);
			pr_err("failed to create tx thread: %d\n", ret);
			goto kthread_run_tx_failed;
		}

		spin_lock_irqsave(&uart->lock, flags);
		uart->rx_task = rx_task;
		uart->tx_task = tx_task;
		spin_unlock_irqrestore(&uart->lock, flags);
	}
	return uart;

kthread_run_tx_failed:
	kthread_stop(rx_task);
kthread_run_rx_failed:
	device_destroy(private.class, uart->dev);
device_create_failed:
	cdev_del(&uart->cdev);
cdev_add_failed:
	unregister_chrdev_region(uart->dev, 1);
alloc_chrdev_region_failed:
	kfree(uart);
	return ERR_PTR(ret);
}

static void mpf_uart_destroy(struct mpf_uart *uart)
{
	dev_dbg(uart->device, "destroying device\n");
	if (uart->tx_task != NULL) {
		dev_dbg(uart->device, "stopping TX thread\n");
		kthread_stop(uart->tx_task);
	}
	if (uart->rx_task != NULL) {
		dev_dbg(uart->device, "stopping RX thread\n");
		kthread_stop(uart->rx_task);
	}
	device_destroy(private.class, uart->dev);
	cdev_del(&uart->cdev);
	unregister_chrdev_region(uart->dev, 1);
	kfree(uart);
}

static unsigned int calc_baudrate(int clk, u32 bval, u32 frac)
{
	unsigned int x = (bval+1)*1000+125*frac;
	uint64_t y = clk;
	y *= 1000;
	y /= x;
	y /= 16;
	return y;
}

static int mpf_uart_probe(struct platform_device *pdev)
{
	int ret, irq;
	void __iomem *base;
	struct resource *res;
	struct mpf_buf *rx, *tx;
	struct mpf_uart *uart;
	struct device *dev = &pdev->dev;

	dev_dbg(&pdev->dev, "probing\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "%s: cannot get resource\n", __func__);
		return -ENXIO;
	}
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		dev_err(&pdev->dev, "%s: ioremap failed: %d\n", __func__, ret);
		return ret;
	} else {
		uint32_t max_size = ior32(base + UART_MAX_SIZE_OFFS);
		uint32_t status = ior32(base + UART_STATUS_OFFS);
		uint32_t error = ior32(base + UART_ERROR_OFFS);
		uint32_t error_count = ior32(base + UART_ERRORCOUNT_OFFS);

		iow32(baud_val, base + UART_BAUDRATE_OFFS);
		iow32(baud_val_fraction, base + UART_FRACT_BAUDRATE_OFFS);
		iow32(uart_params, base + UART_PARAMS_OFFS);
		//iow32(rx_idle_timeout, base + UART_RX_IDLE_TIMEOUT_OFFS);
		//iow32(hold_off_time, base + UART_HOLD_OFF_TIME_OFFS);

		dev_dbg(&pdev->dev, "BAUD_VAL = %#x\n", baud_val);
		dev_dbg(&pdev->dev, "BAUD_VAL_FRACTION = %#x\n", baud_val_fraction);
		dev_dbg(&pdev->dev, "BAUDRATE = %u\n", calc_baudrate(FPGA_CLK, baud_val, baud_val_fraction));
		dev_dbg(&pdev->dev, "UART_PARAMS = %#x\n", uart_params);
		dev_dbg(&pdev->dev, "MAX_SIZE = %u\n", max_size);
		dev_dbg(&pdev->dev, "STATUS = %#x\n", status);
		dev_dbg(&pdev->dev, "ERROR = %#x\n", error);
		dev_dbg(&pdev->dev, "ERROR COUNT = %#x\n", error_count);
		dev_dbg(&pdev->dev, "RX_IDLE_TIMEOUT = %u\n", rx_idle_timeout);
		dev_dbg(&pdev->dev, "HOLD_OFF_TIME = %u\n", hold_off_time);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq >= 0) {
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
			&mpf_uart_isr, IRQF_ONESHOT, dev_name(&pdev->dev), pdev);
		if (ret) {
			dev_err(&pdev->dev, "failed to request irq %d: %d\n",
				irq, ret);
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "failed to get irq resource: %d\n", irq);
	}

	mpf_class = dev->parent->class;
	if (!mpf_class) {
		mpf_class = class_create(THIS_MODULE, DRV_NAME);
		if (IS_ERR(mpf_class)) {
			dev_err(&pdev->dev,
					"%s: cannot create class\n", __func__);
			return PTR_ERR(mpf_class);
		}
	}
	private.class = mpf_class;

	rx = mpf_buf_alloc(buffer_size);
	tx = mpf_buf_alloc(buffer_size);
	uart = mpf_uart_create(DRV_NAME, rx, tx, base);
	if (IS_ERR(uart)) {
		ret = PTR_ERR(uart);
		dev_err(&pdev->dev, "failed to create UART device: %d\n", ret);
		mpf_buf_free(tx);
		mpf_buf_free(rx);
		return ret;
	}
	platform_set_drvdata(pdev, uart);

	return 0;
}

static int mpf_uart_remove(struct platform_device *pdev)
{
	struct mpf_uart *uart = platform_get_drvdata(pdev);
	struct mpf_buf *rx = uart->rx;
	struct mpf_buf *tx = uart->tx;
	struct device *dev = &pdev->dev;

	dev_dbg(&pdev->dev, "removing\n");
	platform_set_drvdata(pdev, NULL);
	mpf_uart_destroy(uart);
	mpf_buf_free(tx);
	mpf_buf_free(rx);
	if (!dev->parent->class) {
		/* We created our own class */
		dev_info(&pdev->dev,
				"%s: destroying class %s.\n", __func__,
				mpf_class->name);
		class_destroy(mpf_class);
	}
	return 0;
}

static struct platform_driver mpf_uart_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= mpf_uart_probe,
	.remove		= mpf_uart_remove,
};

static int __init mpf_uart_init(void)
{
	int ret;

	pr_debug("loading\n");
	INIT_LIST_HEAD(&private.uart_list);

	private.x = mpf_buf_alloc(buffer_size);
	private.y = mpf_buf_alloc(buffer_size);
	private.z = mpf_buf_alloc(buffer_size);

	ret = platform_driver_register(&mpf_uart_driver);
	if (ret)
		pr_err("failed to register driver: %d\n", ret);

	pr_debug("loaded\n");
	return 0;
}
module_init(mpf_uart_init);

static void __exit mpf_uart_exit(void)
{
	pr_debug("unloading\n");
	platform_driver_unregister(&mpf_uart_driver);
	while (!list_empty(&private.uart_list)) {
		struct mpf_uart *uart = list_last_entry(&private.uart_list,
			struct mpf_uart, list);
		list_del(&uart->list);
		mpf_uart_destroy(uart);
	}
	mpf_buf_free(private.z);
	mpf_buf_free(private.y);
	mpf_buf_free(private.x);
	pr_debug("unloaded\n");
}
module_exit(mpf_uart_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Leica Custom UART");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:mpf_uart");

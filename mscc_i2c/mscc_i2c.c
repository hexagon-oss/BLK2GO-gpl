// SPDX-License-Identifier: LGPL-2.0

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/i2c-xiic.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DRV_NAME "mscc_i2c"
#define DRV_VERSION "1.0"

#define USE_POLL

/* CoreI2C control register */
#define COREI2C_CTRL_OFFS 0x00
#define COREI2C_CTRL_CR2 (1 << 7)
#define COREI2C_CTRL_ENS1 (1 << 6)
#define COREI2C_CTRL_STA (1 << 5)
#define COREI2C_CTRL_STO (1 << 4)
#define COREI2C_CTRL_SI (1 << 3)
#define COREI2C_CTRL_AA (1 << 2)
#define COREI2C_CTRL_CR1 (1 << 1)
#define COREI2C_CTRL_CR0 (1 << 0)
#define COREI2C_CTRL_PDIV256 0
#define COREI2C_CTRL_PDIV224 COREI2C_CTRL_CR0
#define COREI2C_CTRL_PDIV192 COREI2C_CTRL_CR1
#define COREI2C_CTRL_PDIV160 (COREI2C_CTRL_CR1 | COREI2C_CTRL_CR0)
#define COREI2C_CTRL_PDIV960 COREI2C_CTRL_CR2
#define COREI2C_CTRL_PDIV120 (COREI2C_CTRL_CR2 | COREI2C_CTRL_CR0)
#define COREI2C_CTRL_PDIV60 (COREI2C_CTRL_CR2 | COREI2C_CTRL_CR1)
#define COREI2C_CTRL_BDIV8 (COREI2C_CTRL_CR2 | COREI2C_CTRL_CR1 \
	| COREI2C_CTRL_CR0)

/* CoreI2C status register */
#define COREI2C_STAT_OFFS 0x04

/* CoreI2C data register */
#define COREI2C_DATA_OFFS 0x08


struct mscc_i2c_dev {
	struct i2c_adapter adap;
	void __iomem *base;
	uint32_t irq_mask;
	struct completion si;
	struct regmap *regmap;
};

static int corei2c_read8(struct device *dev, void __iomem *addr);

/**
 * _wait_for_completion_timeout: Wrapper for polling mode
 * @cp: holds the state of the completion
 * @timeout: timeout value in jiffies
 *
 * Return: 0 if timed out, and positive (at least 1, or number of jiffies left
 * till timeout) if completed.
 */
static int _wait_for_completion_timeout(struct i2c_adapter *adap,
			struct completion *cp, unsigned long timeout)
{
#ifdef USE_POLL
	/* wait for serial interrupt flag (SI) */
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl;
	unsigned long timeo;

	timeo = jiffies + timeout;
	for (;;) {
		ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);
		if (ctrl & COREI2C_CTRL_SI)
			break;

		if (time_after(jiffies, timeo)) {
			dev_err(&adap->dev,
					"waiting for SI flag timed out.\n");
			return 0;
		}
		udelay(100);
	}
	return 1;
#else
	return wait_for_completion_timeout(adap, cp, timeout);
#endif
}

static void corei2c_write8(struct device *dev, int value, void __iomem *addr)
{
	dev_dbg(dev, "%s: %p <= %#x\n", __func__, addr, value);
	iowrite8(value, addr);
}

static int corei2c_read8(struct device *dev, void __iomem *addr)
{
	int value = ioread8(addr);
	dev_dbg(dev, "%s: %p => %#x\n", __func__, addr, value);
	return value;
}

static int put_start_cond(struct i2c_adapter *adap)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);

	dev_dbg(&adap->dev, "%s\n", __func__);
	ctrl |= COREI2C_CTRL_STA;
	ctrl &= ~(COREI2C_CTRL_STO | COREI2C_CTRL_SI);
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	return _wait_for_completion_timeout(adap, &i2c_dev->si, adap->timeout);
}

static int put_repeated_start_cond(struct i2c_adapter *adap)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);

	dev_dbg(&adap->dev, "%s\n", __func__);
	ctrl |= COREI2C_CTRL_STA;
	ctrl &= ~(COREI2C_CTRL_STO | COREI2C_CTRL_SI);
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	return _wait_for_completion_timeout(adap, &i2c_dev->si, adap->timeout);
}

static int put_stop_cond(struct i2c_adapter *adap)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);
	int completed;

	dev_dbg(&adap->dev, "%s\n", __func__);
	ctrl |= COREI2C_CTRL_STO;
	ctrl &= ~(COREI2C_CTRL_STA | COREI2C_CTRL_SI);
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	completed = _wait_for_completion_timeout(adap, &i2c_dev->si, adap->timeout);

	if (completed <= 0)
		dev_err(&adap->dev, "%s: timeout %d\n", __func__, completed);

	ctrl &= ~COREI2C_CTRL_STO;
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	return completed;
}

static int put_addr(struct i2c_adapter *adap, struct i2c_msg *msg)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);
	int addr;

	dev_dbg(&adap->dev, "%s: %#02x\n", __func__, msg->addr);
	addr = ((0x7f & msg->addr) << 1);
	if (msg->flags & I2C_M_RD)
		addr |= 1;

	corei2c_write8(&adap->dev, addr, base + COREI2C_DATA_OFFS);
	ctrl &= ~(COREI2C_CTRL_STA | COREI2C_CTRL_STO | COREI2C_CTRL_SI);
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	return _wait_for_completion_timeout(adap, &i2c_dev->si, adap->timeout);
}

static int put_byte(struct i2c_adapter *adap, u8 data)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);

	dev_dbg(&adap->dev, "%s: %#02x\n", __func__, data);
	corei2c_write8(&adap->dev, data, base + COREI2C_DATA_OFFS);
	ctrl &= ~(COREI2C_CTRL_STA | COREI2C_CTRL_STO | COREI2C_CTRL_SI);
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	return _wait_for_completion_timeout(adap, &i2c_dev->si, adap->timeout);
}

static void get_byte(struct i2c_adapter *adap, u8 *data, int ack)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;

	*data = corei2c_read8(&adap->dev, base + COREI2C_DATA_OFFS);
	dev_dbg(&adap->dev, "%s: %#02x %s\n", __func__, *data,
		ack ? "ACK" : "NACK");
}

static int wait_for_byte_put_ack(struct i2c_adapter *adap, int ack)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS);

	dev_dbg(&adap->dev, "%s: putting %s\n", __func__, ack ? "ACK" : "NACK");

	ctrl &= ~(COREI2C_CTRL_STA | COREI2C_CTRL_STO | COREI2C_CTRL_SI
		| COREI2C_CTRL_AA);

	if (ack)
		ctrl |= COREI2C_CTRL_AA;

	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	return _wait_for_completion_timeout(adap, &i2c_dev->si, adap->timeout);
}

static void corei2c_reset(struct i2c_adapter *adap)
{
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	int ctrl = COREI2C_CTRL_PDIV960;

	dev_dbg(&adap->dev, "%s\n", __func__);
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
	ctrl |= COREI2C_CTRL_ENS1;
	corei2c_write8(&adap->dev, ctrl, base + COREI2C_CTRL_OFFS);
}

static int mscc_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
	int num)
{
	/* xfer function is based on i2c-algo-pca.c */

	int i, j;
	int curmsg;
	int ret;
	u8 state;
	struct mscc_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	void __iomem *base = i2c_dev->base;
	struct i2c_msg *msg;
	unsigned long timeout;
	int completed = 1;
	int numbytes = 0;

	dev_dbg(&adap->dev, "xfer start: num %d\n", num);

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_RD) {
			dev_dbg(&adap->dev,
				"  %d: read 0x%04x slave 0x%04x %d\n", i,
				msgs[i].flags, msgs[i].addr, msgs[i].len);
		} else {
			dev_dbg(&adap->dev,
				"  %d: write 0x%04x slave 0x%04x %d\n",
				i, msgs[i].flags, msgs[i].addr, msgs[i].len);

			for (j = 0; j < msgs[i].len; j++)
				dev_dbg(&adap->dev, "       %#02x\n",
					msgs[i].buf[j]);
		}
	}

	// wait until idle
	timeout = jiffies + adap->timeout;
	while ((state = corei2c_read8(&adap->dev, base + COREI2C_STAT_OFFS)) != 0xf8) {
		if (time_before(jiffies, timeout)) {
			msleep(20);
		} else {
			dev_dbg(&adap->dev, "bus is not idle. status is %#02x\n",
				state);
			return -EBUSY;
		}
	}

	curmsg = 0;
	ret = -EIO;
	while (curmsg < num) {
		state = corei2c_read8(&adap->dev, base + COREI2C_STAT_OFFS);
		dev_dbg(&adap->dev, "state: %#02x\n", state);
		msg = &msgs[curmsg];

		switch (state) {
		case 0xf8: // No relevant state information available; si = 0 (a.k.a. idle)
			completed = put_start_cond(adap);
			break;

		case 0x08: // A START condition has been transmitted.
		case 0x10: // A repeated START condition has been transmitted.
			completed = put_addr(adap, msg);
			break;

		case 0x18: // SLA + W has been transmitted; ACK has been received.
		case 0x28: // Data byte in Data Register has been transmitted; ACK has been received.
			if (numbytes < msg->len) {
				completed = put_byte(adap, msg->buf[numbytes]);
				numbytes++;
				break;
			}
			curmsg++;
			numbytes = 0;
			if (curmsg == num)
				put_stop_cond(adap);
			else
				completed = put_repeated_start_cond(adap);
			break;

		case 0x20: // SLA + W has been transmitted; NACK has been received.
		case 0x48: // SLA + R has been transmitted; NACK has been received.
			dev_err(&adap->dev, "slave not responding\n");
			put_stop_cond(adap);
			ret = -ENXIO;
			goto out;

		case 0x30: // Data byte in Data Register has been transmitted; NACK has been received.
			dev_dbg(&adap->dev, "NACK received\n");
			put_stop_cond(adap);
			goto out;

		case 0x38: // Arbitration lost in SLA + R/W or data bytes.
			dev_dbg(&adap->dev, "Arbitration lost\n");
			completed = put_start_cond(adap);
			goto out;

		case 0x40: // SLA + R has been transmitted; ACK has been received.
			completed = wait_for_byte_put_ack(adap, msg->len > 1);
			break;

		case 0x50: // Data byte has been received; ACK has been returned.
			if (numbytes < msg->len) {
				get_byte(adap, &msg->buf[numbytes], 1);
				numbytes++;
				completed = wait_for_byte_put_ack(adap,
					numbytes < msg->len - 1);
				break;
			}
			curmsg++;
			numbytes = 0;
			if (curmsg == num)
				put_stop_cond(adap);
			else
				completed = put_repeated_start_cond(adap);
			break;

		case 0x58: // Data byte has been received; NACK has been returned.
			if (numbytes == msg->len - 1) {
				get_byte(adap, &msg->buf[numbytes], 0);
				curmsg++;
				numbytes = 0;
				if (curmsg == num)
					put_stop_cond(adap);
				else
					completed = put_repeated_start_cond(adap);
				break;
			}
			dev_err(&adap->dev, "NACK, internal error\n");
			put_stop_cond(adap);
			goto out;

		case 0x00: // Bus error during MST or selected slave modes.
			dev_err(&adap->dev, "Bus error during MST or selected slave modes.\n");
			corei2c_reset(adap);
			goto out;

		default:
			dev_err(&adap->dev, "unhandled state: %#02x", state);
			corei2c_reset(adap);
			goto out;
		}

		if (completed == 0)
			dev_err(&adap->dev, "timeout\n");

		if (!completed)
			goto out;
	}

	ret = curmsg;
out:
	dev_dbg(&adap->dev, "xfer end: %d/%d, status: %#02x, control: %#02x",
		curmsg, num, corei2c_read8(&adap->dev, base + COREI2C_STAT_OFFS),
		corei2c_read8(&adap->dev, base + COREI2C_CTRL_OFFS));

	corei2c_reset(adap);

	return ret;
}

static u32 mscc_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static irqreturn_t mscc_i2c_isr(int irq, void *data)
{
	struct mscc_i2c_dev *i2c_dev = data;

	pr_debug("%s", __func__);

	dev_dbg(&i2c_dev->adap.dev, "%s: complete", __func__);
	complete(&i2c_dev->si);

	return IRQ_HANDLED;
}

static struct i2c_algorithm mscc_i2c_algo = {
	.functionality = &mscc_i2c_func,
	.master_xfer   = &mscc_i2c_xfer,
};

static int mscc_i2c_probe(struct platform_device *pdev)
{
	struct mscc_i2c_dev *i2c_dev;
	int ret, irq;
	void __iomem *base;
	struct resource *res;
	int i;
	struct xiic_i2c_platform_data *pdata;

	dev_dbg(&pdev->dev, "probing\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "%s: cannot get resource\n",
				__func__);
		return -ENXIO;
	}
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		dev_err(&pdev->dev, "%s: ioremap failed: %d\n",
				__func__, ret);
		return ret;
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	i2c_dev->base = base;
	strlcpy(i2c_dev->adap.name, DRV_NAME, sizeof(i2c_dev->adap.name));
	i2c_dev->adap.owner = THIS_MODULE;
	i2c_dev->adap.class = I2C_CLASS_HWMON;
	i2c_dev->adap.algo = &mscc_i2c_algo;
	i2c_dev->adap.dev.parent = &pdev->dev;
	i2c_dev->adap.nr = pdev->id;	/* XXX: use pdev->id */
	i2c_dev->adap.algo_data = i2c_dev;

	i2c_set_adapdata(&i2c_dev->adap, i2c_dev);
	init_completion(&i2c_dev->si);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get irq resource: %d\n", irq);
		return irq;
	}
	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, &mscc_i2c_isr,
				IRQF_ONESHOT, dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq %d: %d\n",
			irq, ret);
		return ret;
	}

	/* set clock divider and enable the CoreI2C IP */
	corei2c_reset(&i2c_dev->adap);

	ret = i2c_add_numbered_adapter(&i2c_dev->adap);
	if (ret) {
		dev_err(&pdev->dev, "failed to add adapter: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, i2c_dev);

	/* Check if there is any i2c_board_info for this bus. */
	pdata = dev_get_platdata(&pdev->dev);
	if (pdata) {
		/* add in known devices to the bus */
		for (i = 0; i < pdata->num_devices; i++)
			i2c_new_device(&i2c_dev->adap, pdata->devices + i);
	}
	dev_info(&pdev->dev, "i2c adapter %d added\n", i2c_dev->adap.nr);
	return 0;
}

static int mscc_i2c_remove(struct platform_device *pdev)
{
	struct mscc_i2c_dev *i2c_dev;
	void __iomem *base;
	u8 ctrl;

	dev_dbg(&pdev->dev, "removing\n");
	i2c_dev = platform_get_drvdata(pdev);
	base = i2c_dev->base;

	/* disable CoreI2C */
	ctrl = corei2c_read8(&pdev->dev, base + COREI2C_CTRL_OFFS);
	ctrl &= ~(COREI2C_CTRL_ENS1 | COREI2C_CTRL_STA | COREI2C_CTRL_STO
		| COREI2C_CTRL_SI | COREI2C_CTRL_AA);
	corei2c_write8(&pdev->dev, ctrl, base + COREI2C_CTRL_OFFS);

	i2c_del_adapter(&i2c_dev->adap);

	return 0;
}

static struct platform_driver mscc_i2c_driver = {
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= mscc_i2c_probe,
	.remove		= mscc_i2c_remove,
};

static int __init mscc_i2c_init(void)
{
	int ret;

	pr_debug("loading\n");
	ret = platform_driver_register(&mscc_i2c_driver);
	if (ret) {
		pr_err("failed to register driver: %d\n", ret);
		return ret;
	}

	pr_info("loaded\n");
	return 0;
}
module_init(mscc_i2c_init);

static void __exit mscc_i2c_exit(void)
{
	pr_debug("unloading\n");
	platform_driver_unregister(&mscc_i2c_driver);
	pr_info("unloaded\n");
}
module_exit(mscc_i2c_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Microsemi CoreI2C Adapter Driver");
MODULE_VERSION(DRV_VERSION);
MODULE_ALIAS("platform:mscc_i2c");

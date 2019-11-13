// SPDX-License-Identifier: GPL-2.0
/*
 * Microsemi/Actel CoreSPI driver
 *
 * The CoreSPI instance is configured at the time of instantiation in hardware
 * design for APB width, frame size, FIFO depth; serial clock speed, serial
 * clock polarity, serial clock phase and slave select state parameters.
 *
 * There are no config register to read back the current configuration.
 */

#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/xilinx_spi.h>	/* xspi_platform_data */

#include "corespi_regs.h"

#define DRV_VERSION "1.1"

/* Configured by the FPGA design. */
#define MPF_SPI_CFG_FIFO_DEPTH	32	/* 32 frames */
/*
 * The driver is not tested with 16 bit words (probably will not work).
 * #define MPF_SPI_BYTES_PER_WORD	2
 */
#define MPF_SPI_BYTES_PER_WORD	1	/* 8 bit per word */

/* define POLL_IO if you want to run the driver in polling mode */
#define POLL_IO

struct mpf_spi_priv {
	struct device		dev;		/* device structure */
	struct spi_master	*master;
	void __iomem		*base;		/* spi registers base address */
	uint32_t		irq_mask;	/* PCIESS irq mask */
	const uint8_t		*tx_buf;	/* tx data buffer */
	uint8_t			*rx_buf;	/* rx data buffer */
	int			tx_len;		/* tx xfer length */
	int			rx_len;		/* rx xfer length */
	int			txerrors;	/* TXFIFO underflow count */
	int			rxerrors;	/* RXFIFO overflow count */
	int			cs;		/* slave device chip select */
	bool			cmd_cont;	/* cs active */
	struct completion	done;		/* completion notification */
};

static inline uint32_t mpf_spi_reg_read(struct mpf_spi_priv *priv, int regoff)
{
	uint32_t val;

	val = readl(priv->base + regoff);
	dev_dbg(&priv->master->dev, "%s: reg %p + %02x = %08x\n", __func__,
			priv->base, regoff, val);

	return val;
}

static inline void mpf_spi_reg_write(struct mpf_spi_priv *priv, int regoff,
				     uint32_t val)
{
	dev_dbg(&priv->master->dev, "%s: reg %p + %02x := %08x\n", __func__,
			priv->base, regoff, val);
	writel(val, priv->base + regoff);
}


static void mpf_spi_disable(struct mpf_spi_priv *priv)
{
	uint32_t reg;

	reg = mpf_spi_reg_read(priv, CTRL1_REG_OFFSET);
	reg &= ~CTRL1_ENABLE_MASK;
	mpf_spi_reg_write(priv, CTRL1_REG_OFFSET, reg);
}

/*
 * This function enables the SPI master controller.
 */
static void mpf_spi_enable(struct mpf_spi_priv *priv)
{
	uint32_t reg;

	reg = mpf_spi_reg_read(priv, CTRL1_REG_OFFSET);
	reg |= CTRL1_ENABLE_MASK | CTRL1_MASTER_MASK;
	mpf_spi_reg_write(priv, CTRL1_REG_OFFSET, reg);
}

#ifndef POLL_IO
static void mpf_spi_disable_irq(struct mpf_spi_priv *priv)
{
	uint32_t reg;

	reg = mpf_spi_reg_read(priv, CTRL1_REG_OFFSET);
	reg &= ~(CTRL1_INTTXDONE_MASK | CTRL1_INTRXOVFLOW_MASK |
			CTRL1_INTTXURUN_MASK);
	mpf_spi_reg_write(priv, CTRL1_REG_OFFSET, reg);

	reg = mpf_spi_reg_read(priv, CTRL2_REG_OFFSET);
	reg &= ~(CTRL2_INTCMD_MASK | CTRL2_INTSSEND_MASK |
			CTRL2_INTRXDATA_MASK | CTRL2_INTTXDATA_MASK);
	mpf_spi_reg_write(priv, CTRL2_REG_OFFSET, reg);
}


static void mpf_spi_enable_irq(struct mpf_spi_priv *priv)
{
	uint32_t reg;

	reg = mpf_spi_reg_read(priv, CTRL1_REG_OFFSET);
	reg |= (CTRL1_INTTXDONE_MASK | CTRL1_INTRXOVFLOW_MASK |
			CTRL1_INTTXURUN_MASK);
	mpf_spi_reg_write(priv, CTRL1_REG_OFFSET, reg);

	reg = mpf_spi_reg_read(priv, CTRL2_REG_OFFSET);
	reg |=  CTRL2_INTRXDATA_MASK;
	mpf_spi_reg_write(priv, CTRL2_REG_OFFSET, reg);
}
#endif


/**
 * mpf_spi_setup - Configure SPI controller for specified transfer
 * @spi:	Pointer to the spi_device structure
 * @transfer:	Pointer to the spi_transfer structure which provides
 *		information about next transfer setup parameters
 *
 * Sets the operational mode of SPI controller for the next SPI transfer
 *
 * Return:	Always 0
 */
static int mpf_spi_setup(struct spi_device *spi)
{
	int cs;
	uint32_t reg;
	struct mpf_spi_priv *priv;

	priv = spi_master_get_devdata(spi->master);

	cs = spi->chip_select;
	priv->dev = spi->dev;
	dev_dbg(&priv->master->dev, "%s: cs %d\n", __func__, cs);

	mpf_spi_disable(priv);

	/* deselect all slaves */
	mpf_spi_reg_write(priv, SSEL_REG_OFFSET, 0);

	/* Flush the receive and transmit FIFOs */
	mpf_spi_reg_write(priv, CMD_REG_OFFSET,
				CMD_TXFIFORST_MASK | CMD_RXFIFORST_MASK);

	/* Clear all interrupts */
	mpf_spi_reg_write(priv, INTCLR_REG_OFFSET, 0xff);

	/* Ensure RXAVAIL, TXRFM, SSEND and CMDINT are disabled */
	mpf_spi_reg_write(priv, CTRL2_REG_OFFSET, 0);

	/*
	 * XXX: hard coded value, calculate from max_speed_hz.
	 * SPICLK = PCLK / (2 * (CLK_DIV + 1))
	 * 125 MHz / (2 * (61 + 1))  is about  1 MHz
	 */
	reg = mpf_spi_reg_read(priv, CLK_DIV_REG_OFFSET);
	dev_dbg(&priv->master->dev,
			"max_speed_hz %u, CLK_DIV old = %02x\n",
			spi->max_speed_hz, reg);
	mpf_spi_reg_write(priv, CLK_DIV_REG_OFFSET, 61);

	/* Select slave. */
	mpf_spi_reg_write(priv, SSEL_REG_OFFSET, (1 << cs));
	mpf_spi_enable(priv);

	dev_dbg(&spi->master->dev, "%s done\n", __func__);

	return 0;
}

static void mpf_spi_read_rxfifo(struct mpf_spi_priv *priv)
{
	uint32_t rx_data;
	unsigned int rxfifo_cnt = 0;
	int i, nbytes;
	int shift;

	dev_dbg(&priv->master->dev, "%s\n", __func__);
	while (!(mpf_spi_reg_read(priv, STATUS_REG_OFFSET)
				& STATUS_RXEMPTY_MASK)
			&& rxfifo_cnt < MPF_SPI_CFG_FIFO_DEPTH) {
		rx_data = mpf_spi_reg_read(priv, RXDATA_REG_OFFSET);
		nbytes = min(priv->rx_len, MPF_SPI_BYTES_PER_WORD);
		if (priv->rx_buf) {
			/*
			 * XXX: check byte order, do we get the last
			 * byte in the upper or lower part of rx_data?
			 */
			/* Assume the last byte is in the upper part. */
			for (i = 0; i < nbytes; i++) {
				shift = BITS_PER_BYTE * i;
				priv->rx_buf[i] = rx_data >> shift;
			}
			priv->rx_buf += nbytes;
		}
		priv->rx_len -= nbytes;
		rxfifo_cnt++;
	}
}

static void mpf_spi_fill_txfifo(struct mpf_spi_priv *priv)
{
	int i;
	uint32_t word, data;
	unsigned int txfifo_cnt = 0;
	int nbytes;
	unsigned int reg = TXDATA_REG_OFFSET;

	dev_dbg(&priv->master->dev, "%s: tx_len %u\n", __func__, priv->tx_len);
	/*
	 * The original CoreSPI source disables the controller before loading
	 * the FIFO, therefore we do it as well.
	 */
	mpf_spi_disable(priv);
	while (priv->tx_len && (txfifo_cnt < MPF_SPI_CFG_FIFO_DEPTH)) {
		word = 0;
		nbytes = min(priv->tx_len, MPF_SPI_BYTES_PER_WORD);

		/* Last word must be written into TXLAST_REG_OFFSET */
		if (priv->tx_len == MPF_SPI_BYTES_PER_WORD)
			reg = TXLAST_REG_OFFSET;

		if (priv->tx_buf) {
			/* XXX: check byte order */
			/* Assume the last byte is in the upper part. */
			for (i = 0; i < nbytes; i++) {
				data = priv->tx_buf[i];
				word |= data << (BITS_PER_BYTE * i);
			}
			priv->tx_buf += nbytes;
		}
		mpf_spi_reg_write(priv, reg, word);
		priv->tx_len -= nbytes;
		txfifo_cnt++;
	}
	mpf_spi_enable(priv);
	dev_dbg(&priv->master->dev, "%s done: tx_len %u\n", __func__,
							priv->tx_len);
}

static irqreturn_t mpf_spi_isr(int irq, void *dev_id)
{
	struct mpf_spi_priv *priv = dev_id;
	uint32_t stat;

	stat = mpf_spi_reg_read(priv, INTMASK_REG_OFFSET);
	if (!stat) {
		/*
		 * XXX: this, stat == 0, actually happened, but I wasn't able
		 * to reproduce it.
		 */
		dev_warn(&priv->dev, "%s: IRQ but status is 0\n", __func__);
		stat = mpf_spi_reg_read(priv, INTRAW_REG_OFFSET);
		dev_warn(&priv->dev, "%s: IRQ raw status 0x%02x\n",
					__func__, stat);
		stat = mpf_spi_reg_read(priv, INTMASK_REG_OFFSET);
		dev_warn(&priv->dev, "%s: IRQ status re-read %02x\n",
					__func__, stat);
		stat = mpf_spi_reg_read(priv, INTMASK_REG_OFFSET);
		dev_warn(&priv->dev, "%s: IRQ status re-read2 %02x\n",
					__func__, stat);

		return IRQ_NONE;
	}

	if (stat & INTMASK_TXDATA_MASK) {
		if (stat & INTMASK_TXUNDERRUN_MASK) {
			dev_warn(&priv->dev, "%s: TX underrun\n", __func__);
			priv->txerrors++;
		}
	}

	if (stat & INTMASK_RXDATA_MASK) {
		if (priv->rx_len)
			mpf_spi_read_rxfifo(priv);
		if (stat & INTMASK_RXOVERFLOW_MASK) {
			dev_warn(&priv->dev, "%s: RX overflow\n", __func__);
			priv->rxerrors++;
		}
	}

	/* write status back to clear interrupts */
	mpf_spi_reg_write(priv, INTCLR_REG_OFFSET, stat);

	if (stat & INTMASK_TXDONE_MASK)
		complete(&priv->done);

	return IRQ_HANDLED;
}


static int mpf_spi_xfer_block(struct  mpf_spi_priv *priv,
		const unsigned char *tx_buf,
		unsigned char *rx_buf, int xfer_len)
{
	int timeout;

	dev_dbg(&priv->master->dev, "%s: xfer_len %u\n", __func__, xfer_len);

	priv->tx_buf = tx_buf;
	priv->rx_buf = rx_buf;
	priv->tx_len = xfer_len;
	priv->rx_len = xfer_len;
	priv->txerrors = priv->rxerrors = 0;

	/* fill TXDATA_FIFO */
	mpf_spi_fill_txfifo(priv);

#ifndef POLL_IO
	/* Enable interrupts. */
	mpf_spi_enable_irq(priv);

	timeout = wait_for_completion_timeout(&priv->done,
				msecs_to_jiffies(1000));
	/* Disable interrupts */
	mpf_spi_disable_irq(priv);
	if (!timeout) {
		dev_err(&priv->dev, "xfer timedout!\n");
		return -ETIMEDOUT;
	}
#else
	timeout = 1000;
	while (priv->rx_len && timeout) {
		mpf_spi_read_rxfifo(priv);
		dev_dbg(&priv->master->dev,
			"%s: remaining rx_len %d, timeout %d\n", __func__,
			priv->rx_len, timeout);
		--timeout;
	}
	if (timeout == 0)
		dev_err(&priv->master->dev, "%s: RX timeout\n", __func__);
#endif

	if (priv->txerrors || priv->rxerrors)
		dev_err(&priv->dev, "Over/Underflow rx %d tx %d xfer %d!\n",
				priv->rxerrors, priv->txerrors, xfer_len);

	dev_dbg(&priv->master->dev,
		"%s done: xfer_len %u, remain %u\n", __func__,
		xfer_len, priv->tx_len);

	return xfer_len - priv->tx_len;
}

static int mpf_spi_txrx_bufs(struct mpf_spi_priv *priv, struct spi_transfer *t)
{
	int bytesleft, sz;
	unsigned char *rx_buf;
	const unsigned char *tx_buf;

	dev_dbg(&priv->master->dev, "%s t->len %u\n", __func__, t->len);
	tx_buf = t->tx_buf;
	rx_buf = t->rx_buf;
	bytesleft = t->len;
	while (bytesleft) {
		sz = mpf_spi_xfer_block(priv, tx_buf, rx_buf, bytesleft);
		if (sz < 0)
			return sz;
		bytesleft -= sz;
		if (tx_buf)
			tx_buf += sz;
		if (rx_buf)
			rx_buf += sz;
	}
	dev_dbg(&priv->master->dev, "%s done: t->len %u\n", __func__, t->len);
	return bytesleft;
}

/**
 * mpf_spi_transfer_one - Initiates the SPI transfer
 * @master:	Pointer to spi_master structure
 * @spi:	Pointer to the spi_device structure
 * @transfer:	Pointer to the spi_transfer structure which provides
 *		information about next transfer parameters
 *
 * This function fills the TX FIFO, starts the SPI transfer and
 * returns 0.
 *
 * Return:	0
 */
static int mpf_spi_transfer_one(struct spi_master *master,
					struct spi_device *spi,
					struct spi_transfer *transfer)
{
	struct mpf_spi_priv *priv = spi_master_get_devdata(master);
	int ret = 0;

	priv->cs = spi->chip_select;
	priv->dev = spi->dev;

	dev_dbg(&priv->dev, "%s, cs %d\n", __func__, priv->cs);

	if (mpf_spi_txrx_bufs(priv, transfer))
		ret = -EIO;

	dev_dbg(&priv->dev, "%s done\n", __func__);
	return ret;
}

/**
 * mpf_spi_prepare - Prepares hardware for transfer.
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 *
 * This function enables SPI master controller.
 *
 * Return:	0 always
 */
static int mpf_spi_prepare(struct spi_master *master)
{
	struct mpf_spi_priv *priv = spi_master_get_devdata(master);

	dev_dbg(&priv->master->dev, "%s\n", __func__);

	mpf_spi_enable(priv);

	return 0;
}

/**
 * mpf_spi_unprepare - Relaxes hardware after transfer
 * @master:	Pointer to the spi_master structure which provides
 *		information about the controller.
 *
 * This function disables the SPI master controller.
 *
 * Return:	0 always
 */
static int mpf_spi_unprepare(struct spi_master *master)
{
	struct mpf_spi_priv *priv = spi_master_get_devdata(master);

	dev_dbg(&priv->dev, "%s\n", __func__);

	mpf_spi_disable(priv);

	return 0;
}


/**
 * mpf_spi_set_cs - Select or deselect the chip select line
 * @spi:	Pointer to the spi_device structure
 * @is_on:	Select(0) or deselect (1) the chip select line
 */
static void mpf_spi_set_cs(struct spi_device *spi, bool is_on)
{
	struct mpf_spi_priv *priv;
	uint32_t reg;
	unsigned int cs;

	cs = spi->chip_select;
	priv = spi_master_get_devdata(spi->master);

	dev_dbg(&priv->master->dev,
			"%s: cs %u, is_on %d\n", __func__, cs, is_on);

	reg = mpf_spi_reg_read(priv, SSEL_REG_OFFSET);

	if (is_on) {
		/* Deselect the slave. */
		reg &= ~(1 << cs);
	} else {
		/* Select the slave. */
		reg |= (1 << cs);
	}

	mpf_spi_reg_write(priv, SSEL_REG_OFFSET, reg);
}

static int mpf_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct mpf_spi_priv *priv;
	struct resource *res;
	int irq, err;
	int i;
	int num_cs = 1;
	int bits_per_word = 8;
	struct xspi_platform_data *pdata;

	dev_info(&pdev->dev, "%s, pdev->id %d\n", __func__, pdev->id);
	master = spi_alloc_master(&pdev->dev, sizeof(*priv));
	if (!master) {
		dev_err(&pdev->dev, "could not alloc master\n");
		return -ENOMEM;
	}

	pdata = dev_get_platdata(&pdev->dev);
	if (pdata) {
		num_cs = pdata->num_chipselect;
		bits_per_word = pdata->bits_per_word;
	}

	priv = spi_master_get_devdata(master);
	priv->master = master;
	platform_set_drvdata(pdev, master);

	master->bus_num = pdev->id;
	master->num_chipselect = num_cs;
	master->bits_per_word_mask = SPI_BPW_MASK(bits_per_word);
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->prepare_transfer_hardware = mpf_spi_prepare;
	master->setup = mpf_spi_setup;
	master->transfer_one = mpf_spi_transfer_one;
	master->unprepare_transfer_hardware = mpf_spi_unprepare;
	master->set_cs = mpf_spi_set_cs;
	master->rt = 1;	/* run message pump with realtime priority */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_info(&pdev->dev, "mem resource %pR\n", res);
	if (!res) {
		dev_err(&pdev->dev, "%s: cannot get spi mem resource\n",
				__func__);
		return -ENXIO;
	}

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base)) {
		err = PTR_ERR(priv->base);
		dev_err(&pdev->dev, "%s: cannot ioremap spi regs: %d\n",
				__func__, err);
		return err;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no IRQ resource found: %d\n", irq);
		return irq;
	}
	err = devm_request_threaded_irq(&pdev->dev, irq, NULL, mpf_spi_isr, 0,
			pdev->name, priv);
	if (err) {
		dev_err(&pdev->dev, "unable to request irq %d\n", irq);
		return err;
	}

	init_completion(&priv->done);

	spi_master_set_devdata(master, priv);

	/* register spi controller */
	err = devm_spi_register_master(&pdev->dev, master);
	if (err) {
		dev_err(&pdev->dev, "spi register master failed!\n");
		spi_master_put(master);
		return err;
	}

	/* create devices */
	if (pdata) {
		for (i = 0; i < pdata->num_devices; i++)
			spi_new_device(master, pdata->devices + i);
	}

	dev_info(&pdev->dev, "probe done\n");

	return 0;
}

static int mpf_spi_remove(struct platform_device *pdev)
{

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id mpf_spi_dt_id[] = {
	{ .compatible = "lgs,mpf-spi" },
	{ },
};
MODULE_DEVICE_TABLE(of, mpf_spi_dt_id);

static struct platform_driver mpf_spi_driver = {
	.probe	= mpf_spi_probe,
	.remove = mpf_spi_remove,
	.driver = {
		.name	= "mpf-spi",
		.of_match_table = mpf_spi_dt_id,
	},
};
module_platform_driver(mpf_spi_driver);

MODULE_AUTHOR("Michael Brandt <michael.brandt@devtwig.se>");
MODULE_DESCRIPTION("Driver for Microsemi/Actel CoreSPI on Polarfire");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL v2");

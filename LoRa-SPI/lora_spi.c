#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/compat.h>
#include <linux/acpi.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/errno.h>
#include <linux/delay.h>

#include "lora_spi.h"
#include "sx1278.h"

#define __DRIVER_NAME		"lora-spi"
#ifndef N_LORASPI_MINORS
#define N_LORASPI_MINORS	8
#endif

static DECLARE_BITMAP(minors, N_LORASPI_MINORS);

static DEFINE_MUTEX(minors_lock);

static ssize_t loraspi_read(struct lora_data *lrdata, const char __user *buf, size_t size) {
	int status;
	struct spi_device *spi;
	int c = 0;
	uint8_t base_adr;
	uint8_t flag;
	uint32_t timeout;

	spi = lrdata->lora_device;
	printk(KERN_DEBUG "lora-spi: SPI device #%d.%d read\n",
		   spi->master->bus_num, spi->chip_select);

	mutex_lock(&(lrdata->buf_lock));
	/* Set chip to standby state. */
	printk(KERN_DEBUG "lora-spi: Going to set standby state\n");
	sx127X_setState(spi, SX127X_STANDBY_MODE);

	/* Set chip FIFO RX base. */
	base_adr = 0x00;
	printk(KERN_DEBUG "lora-spi: Going to set RX base address\n");
	sx127X_write_reg(spi, SX127X_REG_FIFO_RX_BASE_ADDR, &base_adr, 1);
	sx127X_write_reg(spi, SX127X_REG_FIFO_ADDR_PTR, &base_adr, 1);

	/* Set chip wait for LoRa timeout time. */
	sx127X_setLoRaRXTimeout(spi, 300);
	/* Clear all of the IRQ flags. */
	sx127X_clearLoRaAllFlag(spi);
	/* Set chip to RX continuous state.  The chip start to wait for receiving. */
	sx127X_setState(spi, SX127X_RXCONTINUOUS_MODE);
	/* Wait and check there is any packet received ready. */
	for(timeout = 0; timeout < 500; timeout++) {
		flag = sx127X_getLoRaFlag(spi, SX127X_FLAG_RXTIMEOUT | SX127X_FLAG_RXDONE | SX127X_FLAG_PAYLOADCRCERROR);
		//printk(KERN_DEBUG "lora-spi: LoRa flag in receiving is %X\n", flag);
		if(flag == 0) mdelay(10);
		else break;
	}

	/* If there is nothing or received timeout. */
	if((flag == 0) || (flag & SX127X_FLAG_RXTIMEOUT)) {
		c = -1;
	}
	/* If there is a packet, but the payload is CRC error. */
	if(sx127X_getLoRaFlag(spi, SX127X_FLAG_PAYLOADCRCERROR)) {
		c = -2;
	}

	/* There is a ready packet in the chip's FIFO. */
	if(c == 0) {
		memset(lrdata->rx_buf, 0, lrdata->bufmaxlen);
		size = (lrdata->bufmaxlen < size) ? lrdata->bufmaxlen : size;
		/* Read from chip to LoRa data RX buffer. */
		c = sx127X_readLoRaData(spi, lrdata->rx_buf, size);
		/* Copy from LoRa data RX buffer to user space. */
		if(c > 0)
			copy_to_user((void *)buf, lrdata->rx_buf, c);
	}

	/* Clear all of the IRQ flags. */
	sx127X_clearLoRaAllFlag(spi);

	/* Set chip to standby state. */
	sx127X_setState(spi, SX127X_SLEEP_MODE);

	mutex_unlock(&(lrdata->buf_lock));

	return c;
}

static ssize_t loraspi_write(struct lora_data *lrdata, const char __user *buf, size_t size) {
	uint8_t c;
	ssize_t status;
	struct spi_device *spi;
	uint8_t data;
	uint8_t base_adr;
	uint8_t flag;
	uint32_t timeout;

	struct lora_packet lrpack;

	spi = lrdata->lora_device;
	printk(KERN_DEBUG "lora-spi: SPI device #%d.%d write %u bytes from user space\n",
		   spi->master->bus_num, spi->chip_select, size);

	mutex_lock(&(lrdata->buf_lock));
	memset(lrdata->tx_buf, 0, lrdata->bufmaxlen);
	status = copy_from_user(lrdata->tx_buf, buf, size);

	if(status >= size)
		return 0;

	lrdata->tx_buflen = size - status;

//	/* Initial LoRa packet. */
//	lrpack.dst = 2;
//	lrpack.src = lrdata->node_adr;
//	lrpack.packet_num = lrdata->packet_num;
//	lrdata->packet_num += 1;
//	memcpy(lrpack.data, lrdata->tx_buf, lrdata->tx_buflen);
//	lrpack.len = lrdata->tx_buflen;

	/* Set chip to standby state. */
	printk(KERN_DEBUG "lora-spi: Going to set standby state\n");
	sx127X_setState(spi, SX127X_STANDBY_MODE);
	data = sx127X_readMode(spi);
	printk(KERN_DEBUG "lora-spi: Current OP mode is 0x%X\n", data);

	/* Set chip FIFO pointer to FIFO TX base. */
	base_adr = 0x80;
	printk(KERN_DEBUG "lora-spi: Going to set TX base address\n");
	sx127X_write_reg(spi, SX127X_REG_FIFO_TX_BASE_ADDR, &base_adr, 1);
	printk(KERN_DEBUG "lora-spi: Going to set FIFO pointer to TX base address 0x%X\n", base_adr);
	sx127X_write_reg(spi, SX127X_REG_FIFO_ADDR_PTR, &base_adr, 1);

	/* Write to SPI chip synchronously to fill the FIFO of the chip. */
//	/* Write LoRa packet destination. */
//	sx127X_write_reg(spi, SX127X_REG_FIFO, &lrpack.dst, 1);
//	/* Write LoRa packet source. */
//	sx127X_write_reg(spi, SX127X_REG_FIFO, &lrpack.src, 1);
//	/* Write LoRa packet number. */
//	sx127X_write_reg(spi, SX127X_REG_FIFO, &lrpack.packet_num, 1);
//	/* Write LoRa packet payload length. */
//	sx127X_write_reg(spi, SX127X_REG_FIFO, &lrpack.len, 1);
//	/* Write LoRa packet payload. */
	//printk(KERN_DEBUG "lora-spi: write %d bytes to chip\n", lrpack.len);
	printk(KERN_DEBUG "lora-spi: write %d bytes to chip\n", lrdata->tx_buflen);
	c = sx127X_write_reg(spi, SX127X_REG_FIFO, lrdata->tx_buf, lrdata->tx_buflen);
//	/* Write LoRa packet retry. */
//	sx127X_write_reg(spi, SX127X_REG_FIFO, &lrpack.retry, 1);

	/* Set the FIFO payload length. */
	data = c;
	printk(KERN_DEBUG "lora-spi: set payload length %d\n", data);
	sx127X_write_reg(spi, SX127X_REG_PAYLOAD_LENGTH, &data, 1);
	sx127X_read_reg(spi, SX127X_REG_PAYLOAD_LENGTH, &data, 1);
	printk(KERN_DEBUG "lora-spi: read payload length %d btyes\n", data);

	sx127X_read_reg(spi, SX127X_REG_FIFO_ADDR_PTR, &data, 1);
	printk(KERN_DEBUG "lora-spi: FIFO address is 0x%X now\n", data);

	/* Clear LoRa IRQ TX flag. */
	sx127X_clearLoRaFlag(spi, SX127X_FLAG_TXDONE);
	printk(KERN_DEBUG "lora-spi: IRQ flags' state: 0x%X\n", sx127X_getLoRaAllFlag(spi));

	if(c > 0) {
		/* Set chip to transmit(TX) state to send the data in FIFO to RF. */
		printk(KERN_DEBUG "lora-spi: set TX state\n");
		sx127X_setState(spi, SX127X_TX_MODE);

		timeout = (c + sx127X_getLoRaPreambleLen(spi) + 1) + 2;
		printk(KERN_DEBUG "lora-spi: the time out is %d us", timeout * 1000);

		/* Wait until TX is finished by checking the TX flag. */
		for(flag = 0; timeout > 0; timeout--) {
			flag = sx127X_getLoRaAllFlag(spi);//, SX127X_FLAG_TXDONE);
			//printk(KERN_DEBUG "lora-spi: wait TX flag is %X\n", flag);
			if((flag & SX127X_FLAG_TXDONE) != 0) {
				printk(KERN_DEBUG "lora-spi: wait TX is finished\n");
				break;
			}
			else {
				if(timeout == 1) {
					c = 0;
					printk(KERN_DEBUG "lora-spi: wait TX is time out\n");
				}
				else {
					udelay(1000);
				}
			}
		}
	}

	/* Set chip to standby state. */
	printk(KERN_DEBUG "lora-spi: set back to standby state\n");
	sx127X_setState(spi, SX127X_STANDBY_MODE);

	lrdata->tx_buflen = 0;

	mutex_unlock(&(lrdata->buf_lock));

	return c;
}

struct lora_driver lr_driver = {
	.name = __DRIVER_NAME,
	.num = N_LORASPI_MINORS,
	.owner = THIS_MODULE,
};

struct lora_operations lrops = {
	.read = loraspi_read,
	.write = loraspi_write,
};

/* The compatible SoC array. */
#ifdef CONFIG_OF
static const struct of_device_id lora_dt_ids[] = {
	{ .compatible = "semtech,sx1278" },
	{ .compatible = "lora-spi" },
	{}, /* Should be terminated with a NULL entry. */
};
MODULE_DEVICE_TABLE(of, lora_dt_ids);
#endif

#ifdef CONFIG_ACPI

/* The compatible ACPI device array. */
#define LORA_ACPI_DUMMY	1
static const struct acpi_device_id lora_acpi_ids[] = {
	{ .id = "lora-spi" },
	{}, /* Should be terminated with a NULL entry. */
};
MODULE_DEVICE_TABLE(acpi, lora_acpi_ids);

/* The callback function of ACPI probes LoRa SPI. */
static void loraspi_probe_acpi(struct spi_device *spi) {
	const struct acpi_device_id *id;

	if(!has_acpi_companion(&(spi->dev)))
		return;

	id = acpi_match_device(lora_acpi_ids, &(spi->dev));
	if(WARN_ON(!id))
		return;

	if(id->driver_data == LORA_ACPI_DUMMY)
		dev_warn(&(spi->dev), "do not use this driver in produciton systems.\n");
}
#else
static void loraspi_probe_acpi(struct spi_device *spi) {};
#endif

/* The compatible SPI device id array. */
static const struct spi_device_id spi_ids[] = {
	{ .name = "lora-spi" },
	{}, /* Should be terminated with a NULL entry. */
};
MODULE_DEVICE_TABLE(spi, spi_ids);

/* The SPI probe callback function. */
static int loraspi_probe(struct spi_device *spi) {
	struct lora_data *lrdata;
	struct device *dev;
	unsigned long minor;
	int status;

	printk(KERN_DEBUG "lora-spi: probe a SPI device with address %d.%d\n",
		   spi->master->bus_num, spi->chip_select);

#ifdef CONFIG_OF
	if(spi->dev.of_node && !of_match_device(lora_dt_ids, &(spi->dev))) {
		dev_err(&(spi->dev), "buggy DT: lora listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
				!of_match_device(lora_dt_ids, (&spi->dev)));
	}
#endif

	loraspi_probe_acpi(spi);

	/* Allocate lora device's data. */
	lrdata = kzalloc(sizeof(struct lora_data), GFP_KERNEL);
	if(!lrdata)
		return -ENOMEM;

	/* Initial the lora device's data. */
	lrdata->lora_device = spi;
	lrdata->ops = &lrops;
	lrdata->node_adr = 1 + spi->chip_select;
	lrdata->packet_num = 0;
	mutex_init(&(lrdata->buf_lock));
	mutex_lock(&minors_lock);
	minor = find_first_zero_bit(minors, N_LORASPI_MINORS);
	if(minor < N_LORASPI_MINORS) {
		set_bit(minor, minors);
		lrdata->devt = MKDEV(lr_driver.major, minor);
		dev = device_create(lr_driver.lora_class,
							&(spi->dev),
							lrdata->devt,
							lrdata,
							"loraSPI%d.%d", spi->master->bus_num, spi->chip_select);
		/* Set the SPI device's driver data for later use.  */
		spi_set_drvdata(spi, lrdata);
		lora_device_add(lrdata);
		status = PTR_ERR_OR_ZERO(dev);
	}
	else {
		/* No more lora device available. */
		kfree(lrdata);
		status = -ENODEV;
	}
	
	/* Initial the SX127X chip. */
	init_sx127X(spi);
	
	mutex_unlock(&minors_lock);

	return status;
}

/* The SPI remove callback function. */
static int loraspi_remove(struct spi_device *spi) {
	struct lora_data *lrdata;
	
	printk(KERN_DEBUG "lora-spi: remove a SPI device with address %d.%d",
		   spi->master->bus_num, spi->chip_select);

	lrdata = spi_get_drvdata(spi);

	/* Clear the lora device's data. */
	lrdata->lora_device = NULL;
	/* No more operations to the lora device from user space. */
	lora_device_remove(lrdata);
	mutex_lock(&minors_lock);
	device_destroy(lr_driver.lora_class, lrdata->devt);
	clear_bit(MINOR(lrdata->devt), minors);
	/* Set the SX127X chip to sleep. */
	sx127X_setState(spi, SX127X_SLEEP_MODE);
	mutex_unlock(&minors_lock);

	/* Free the memory of the lora device.  */
	kfree(lrdata);
	
	return 0;
}

/* The SPI driver which acts as a protocol driver in this kernel module. */
static struct spi_driver lora_spi_driver = {
	.driver = {
		.name = __DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = lora_dt_ids,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(lora_acpi_ids),
#endif
	},
	.probe = loraspi_probe,
	.remove = loraspi_remove,
	.id_table = spi_ids,
};


/* UseExample kernel module's initial function. */
static int loraspi_init(void) {
	int status;
	
	printk(KERN_DEBUG "lora-spi: init SX1278 compatible kernel module\n");
	
	/* Register a kind of LoRa driver. */
	lora_register_driver(&lr_driver);

	/* Register LoRa SPI driver as an SPI driver. */
	status = spi_register_driver(&lora_spi_driver);

	return status;
}

/* UseExample kernel module's exit function. */
static void loraspi_exit(void) {
	printk(KERN_DEBUG "lora-spi: exit\n");

	/* Unregister the LoRa SPI driver. */
	spi_unregister_driver(&lora_spi_driver);
	/* Unregister the lora driver. */
	lora_unregister_driver(&lr_driver);
}

module_init(loraspi_init);
module_exit(loraspi_exit);

MODULE_LICENSE("Dual BSD/GPL");

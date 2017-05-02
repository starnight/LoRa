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

#include "lora_spi.h"
#include "sx1278.h"

#define __DRIVER_NAME		"lora-spi"
#define N_LORASPI_MINORS	4

static DECLARE_BITMAP(minors, N_LORASPI_MINORS);

static DEFINE_MUTEX(minors_lock);

static ssize_t loraspi_read(struct lora_data *lrdata, const char __user *buf, size_t size) {
	size_t len = 0, oblen;
	int res;
	struct spi_device *spi;

	spi = lrdata->lora_device;
	printk(KERN_DEBUG "lora-spi: SPI device #%d.%d read\n",
		   spi->master->bus_num, spi->chip_select);

//	mutex_lock(&(lrdata->buf_lock));
//	/* Read from the SPI device into the lora data's RX buffer. */
//	res = lrdata->bufmaxlen - lrdata->rx_buflen;
//	len = (res <= size) ? res : size;
//	len = somedevice_read(spi, lrdata->rx_buf + lrdata->rx_buflen, len);
//	oblen = lrdata->rx_buflen;
//	lrdata->rx_buflen += len;
//
//	/* Read from the lora data into user space. */
//	copy_to_user((void *)buf, lrdata->rx_buf + oblen, len);
//	lrdata->rx_buflen -= len;
//
//	mutex_unlock(&(lrdata->buf_lock));

	return len;
}

static ssize_t loraspi_write(struct lora_data *lrdata, const char __user *buf, size_t size) {
	ssize_t c = 0;
	size_t len, oblen;
	int res;
	struct spi_device *spi;

	spi = lrdata->lora_device;
	printk(KERN_DEBUG "lora-spi: SPI device #%d.%d write\n",
		   spi->master->bus_num, spi->chip_select);

//	mutex_lock(&(lrdata->buf_lock));
//	while(size > 0) {
//		/* Write from user space into the lora data. */
//		res = lrdata->bufmaxlen - lrdata->tx_buflen;
//		len = (res <= size) ? res : size;
//		if(len == 0)
//			break;
//		copy_from_user(lrdata->tx_buf + lrdata->tx_buflen, buf + c, len);
//		oblen = lrdata->tx_buflen;
//		lrdata->tx_buflen += len;
//
//		/* Write from lora data into SPI device. */
//		res = somedevice_write(spi, lrdata->tx_buf + oblen, lrdata->tx_buflen);
//		lrdata->tx_buflen -= len;
//		if(res <= 0) {
//			break;
//		}
//		else if(res < len) {
//			c += res;
//			break;
//		}
//		c += len;
//		size -= len;
//	}
//	mutex_unlock(&(lrdata->buf_lock));

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
	
	/* Initial the SX1278 chip. */
	init_sx1278(spi);
	
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
	/* Set the SX1278 chip to sleep. */
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
	
	printk(KERN_DEBUG "lora-spi: init\n");
	
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

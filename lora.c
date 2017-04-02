#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include "test_ioctl.h"

MODULE_LICENSE("Dual BSD/GPL");

#define LORA_NAME	"lora"

/* The device packaged data and registers structure. */
struct device_data {
	dev_t devt;
	spinlock_t spi_lock;
	struct spi_device *spi;
	struct list_head device_entry;

	struct mutex buf_lock;
	uint16_t users;
	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
	uint32_t speed_hz;

	uint32_t flag;
	uint8_t *data;
	uint32_t len;
	rwlock_t lock;
};

static unsigned int lora_major = 0;
static unsigned int lora_devs = 2;
static struct cdev lora_cdev;
static struct class *lora_sys_class = NULL;



/* The device initial data and registers' value. */
static char _buffer[] = "0123456789\r\n";
static struct device_data dev_data = {
									.flag = 0,
									.data = _buffer,
									.len = 12};

static int lora_open(struct inode *inode, struct file *filp) {
	printk(KERN_DEBUG "lora: open\n");

	/* Map the data location to the file data pointer. */
	filp->private_data = &dev_data;
	/* Initial lock. */
	rwlock_init(&(dev_data.lock));

	return 0;
}

static int lora_close(struct inode *inode, struct file *filp) {
	printk(KERN_DEBUG "lora: close\n");
	
	/* Release the mapping of file data address. */
	if(filp->private_data) {
		filp->private_data = NULL;
	}

	return 0;
}

static ssize_t lora_read(struct file *filp, char __user *buf, size_t size, loff_t *f_pos) {
	size_t count;
	uint8_t byte;
	struct device_data *data_p;
	
	printk(KERN_DEBUG "lora: read (size=%zu)\n", size);

	data_p = filp->private_data;
	/* Get the lock for reading. */
	read_lock(&(data_p->lock));
	/* Read from the device data to user space. */
	for(count = 0; (count < size) && (*f_pos) < data_p->len; ++(*f_pos), ++count) {
		byte = data_p->data[*f_pos];
		if(copy_to_user(buf + count, &byte, 1) != 0) {
			break;
		}
		printk(KERN_DEBUG "lora: read (buf[%zu]=%02x)\n", count, (unsigned)byte);
	}
	/* Release the lock for reading. */
	read_unlock(&(data_p->lock));

	return count;
}

static ssize_t lora_write(struct file *filp, const char __user *buf, size_t size, loff_t *f_pos) {
	size_t count;
	ssize_t ret;
	uint8_t byte;
	struct device_data *data_p;

	printk(KERN_DEBUG "lora: write (size=%zu)\n", size);

	data_p = filp->private_data;
	/* Get the lock for writing. */
	write_lock(&(data_p->lock));
	/* Write from user space to the device. */
	for(count = 0; (count < size) && (*f_pos) < data_p->len; ++(*f_pos), ++count) {
		if(copy_from_user(&byte, buf + count, 1) != 0) {
			break;
		}
		data_p->data[*f_pos] = byte;
		printk(KERN_DEBUG "lora: write (buf[%zu]=%02x)\n", count, (unsigned)byte);
	}
	/* Release the lock for writing. */
	write_unlock(&(data_p->lock));

	if((count == 0) && ((*f_pos) >= data_p->len)) {
		ret = -ENOBUFS;
	}
	else {
		ret = count;
	}

	return ret;
}

static long lora_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	long ret;
	int *pval;
	struct device_data *data_p;

	ret = 0;
	pval = (int __user *)arg;

	printk(KERN_DEBUG "lora: unlocked_ioctl (cmd=%d)\n", cmd);

	data_p = filp->private_data;

	/* I/O control by each command. */
	switch(cmd) {
		case IOCTL_GET_FLAG:
			if(copy_to_user(pval, &(data_p->flag), sizeof(data_p->flag)))
				ret = -EFAULT;
			else
				printk(KERN_DEBUG "lora: ioctl get flag %d\n", data_p->flag);
			break;
		case IOCTL_SET_FLAG:
			if(copy_from_user(&(data_p->flag), pval, sizeof(data_p->flag)))
				ret = -EFAULT;
			else
				printk(KERN_DEBUG "lora: ioctl set flag %d\n", data_p->flag);
			break;
		default:
			ret = -ENOTTY;
	}

	return ret;
}

static struct file_operations lora_fops = {
	.open = lora_open,
	.release = lora_close,
	//.read = lora_read,
	//.write = lora_write,
	//.unlocked_ioctl = lora_ioctl,
};

//#ifdef CONFIG_OF
static const struct of_device_id lora_dt_ids[] = {
	{ .compatible = "semtech,sx1278" },
	{ .compatible = "rpi-lora-spi" },
	{ .compatible = "brcm,bcm2835-spi" },
	{}, /* Should be terminated with a NULL entry. */
};
MODULE_DEVICE_TABLE(of, lora_dt_ids);
//#endif

#ifdef CONFIG_ACPI
#define LORA_ACPI_DUMMY	1
static const struct acpi_device_id lora_acpi_ids[] = {
	{"SPT0001", LORA_ACPI_DUMMY},
	{}, /* Should be terminated with a NULL entry. */
};
MODULE_DEVICE_TABLE(acpi, lora_acpi_ids);

static void lora_probe_acpi(struct spi_device *spi) {
	const struct acpi_device_id *id;

	if(!has_acpi_companion(&(spi->dev)))
		return;

	id = acpi_match_table(lora_acpi_ids, &(spi->dev));
	if(WARN_ON(!id))
		return;

	if(id->driver_data == LORA_ACPI_DUMMY)
		dev_warn(&(spi->dev), "do not use this driver in produciton systems.\n");
}
#else
static void lora_probe_acpi(struct spi_device *spi) {};
#endif

#define N_SPI_MINORS	2

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);

static int lora_spi_probe(struct spi_device *spi) {
	struct device_data *lora_data;
	struct device *dev;
	int status;
	unsigned long minor;

	printk(KERN_DEBUG "lora: probe lora spi.\n");
	if(spi->dev.of_node && !of_match_device(lora_dt_ids, &(spi->dev))) {
		dev_err(&(spi->dev), "buggy DT: lora listed directly in DT\n");
		WARN_ON(spi->dev.of_node &&
				!of_match_device(lora_dt_ids, (&spi->dev)));
	}

	lora_probe_acpi(spi);

	/* Allocate driver data. */
	lora_data = (struct device_data *)kzalloc(sizeof(lora_data), GFP_KERNEL);
	if(!lora_data)
		return -ENOMEM;

	/* Inital the driver data. */
	lora_data->spi = spi;


	INIT_LIST_HEAD(&(lora_data->device_entry));
	
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if(minor < N_SPI_MINORS) {
		lora_data->devt = MKDEV(lora_major, minor);
		dev = device_create(lora_sys_class, &(spi->dev), lora_data->devt,
							lora_data, "loraSPI%d.%d",
							spi->master->bus_num, spi->chip_select);
		status = PTR_ERR_OR_ZERO(dev);
	}
	else {
		dev_dbg(&spi->dev, "no minor number available.\n");
		status = -ENODEV;
	}
	if(status == 0) {
		set_bit(minor, minors);
		list_add(&lora_data->device_entry, &device_list);
	}

	lora_data->speed_hz = spi->max_speed_hz;

	if(status == 0) {
		spi_set_drvdata(spi, lora_data);
		printk(KERN_DEBUG "lora: loraSPI%d.%d device node created.\n",
						  spi->master->bus_num, spi->chip_select);
	}
	else
		kfree(lora_data);

	return status;
}

static int lora_spi_remove(struct spi_device *spi) {
	struct device_data *lora_data = spi_get_drvdata(spi);

	lora_data->spi = NULL;

	list_del(&(lora_data->device_entry));
	clear_bit(MINOR(lora_data->devt), minors);
	device_destroy(lora_sys_class, lora_data->devt);
	printk(KERN_DEBUG "lora: loraSPI%d.%d device node deleted.\n",
						spi->master->bus_num, spi->chip_select);
	
	if(lora_data->users == 0)
		kfree(lora_data);

	return 0;
}


static struct spi_driver lora_spi_driver = {
	.driver = {
		.name = "rpi-lora-spi",
		.owner = THIS_MODULE,
//#ifdef CONFIG_OF
		.of_match_table = lora_dt_ids,
//#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(lora_acpi_ids),
#endif
	},
	.probe = lora_spi_probe,
	.remove = lora_spi_remove,
};




static int lora_init(void) {
	dev_t dev = MKDEV(lora_major, 0);
	int alloc_ret, cdev_err, status;

	printk(KERN_DEBUG "lora: init\n");

	/* Allocate a character device. */
	alloc_ret = alloc_chrdev_region(&dev, 0, lora_devs, LORA_NAME);
	if(alloc_ret) {
		printk(KERN_DEBUG "lora: Failed to allocate a character device\n");
		return -1;
	}
	/* Initial the character device driver. */
	lora_major = MAJOR(dev);
	cdev_init(&lora_cdev, &lora_fops);
	lora_cdev.owner = THIS_MODULE;
	/* Add the character device driver into system. */
	cdev_err = cdev_add(&lora_cdev, MKDEV(lora_major, 0), lora_devs);
	if(cdev_err) {
		printk(KERN_DEBUG "lora: Failed to register a character device\n");
		/* Release the allocated character device. */
		if(alloc_ret == 0) {
			unregister_chrdev_region(dev, lora_devs);
		}
		return -1;
	}
	printk(KERN_DEBUG "lora: %s driver(major %d) installed.\n", LORA_NAME, lora_major);

	/* Create device class. */
	lora_sys_class = class_create(THIS_MODULE, LORA_NAME);
	if(IS_ERR(lora_sys_class)) {
		printk(KERN_DEBUG "lora: Failed to create a class of device.\n");
		/* Release the added character device. */
		if(cdev_err == 0)
			cdev_del(&lora_cdev);
		/* Release the allocated character device. */
		if(alloc_ret == 0)
			unregister_chrdev_region(dev, lora_devs);
		return -1;
	}
	printk(KERN_DEBUG "lora: %s class created.\n", LORA_NAME);

	/* Register LoRa SPI driver. */
	status = spi_register_driver(&lora_spi_driver);
	if(status < 0) {
		printk(KERN_DEBUG "lora: Failed to register a LoRa SPI driver.\n");
		/* Delete device class. */
		class_destroy(lora_sys_class);
		/* Delete the character device driver from system. */
		cdev_del(&lora_cdev);
		/* Unregister the allocated character device. */
		unregister_chrdev_region(dev, lora_devs);
	}
	else {
		printk(KERN_DEBUG "lora: %s driver registered.\n", LORA_NAME);
	}

	/* Create device node. */
	//device_create(lora_sys_class, NULL, dev, NULL, LORA_NAME);
	//printk(KERN_DEBUG "lora: %s device node created.\n", LORA_NAME);

	return status;
}

static void lora_exit(void) {
	dev_t dev = MKDEV(lora_major, 0);

	printk(KERN_DEBUG "lora: exit\n");
	/* Destory device nodes. */
	//device_destroy(lora_sys_class, dev);
	/* Delete device class. */
	class_destroy(lora_sys_class);
	/* Delete the character device driver from system. */
	cdev_del(&lora_cdev);
	/* Unregister the allocated character device. */
	unregister_chrdev_region(dev, lora_devs);
	printk(KERN_DEBUG "lora: %s driver removed.\n", LORA_NAME); 
}

module_init(lora_init);
module_exit(lora_exit);

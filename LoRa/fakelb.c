/*-
 * Copyright (c) 2017 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/spi/spi.h>
#include <net/mac802154.h>
#include <net/cfg802154.h>

static LIST_HEAD(fakelb_ifup_phys);
static DEFINE_RWLOCK(fakelb_ifup_phys_lock);

static LIST_HEAD(fakelb_vbufs);

#define FAKELB_RXDONE	1
#define FAKELB_TXDONE	2

struct virtual_wpan_buf {
	struct ieee802154_hw *hw;
	struct list_head list_vbuf;

	uint8_t buf[IEEE802154_MTU];
	uint8_t len;
};

struct sx1278_phy {
	struct ieee802154_hw *hw;
	void *intf;

	u8 page;
	u8 channel;

	bool suspended;
	u8 flags;
	struct timer_list timer;
	struct work_struct irqwork;
	struct sk_buff *txbuf;
	spinlock_t buf_lock;

	struct list_head list_ifup;
};

static int fakelb_hw_ed(struct ieee802154_hw *hw, u8 *level)
{
	BUG_ON(!level);
	*level = 0xbe;

	return 0;
}

static int fakelb_hw_channel(struct ieee802154_hw *hw, u8 page, u8 channel)
{
	struct sx1278_phy *phy = hw->priv;

	write_lock_bh(&fakelb_ifup_phys_lock);
	phy->page = page;
	phy->channel = channel;
	write_unlock_bh(&fakelb_ifup_phys_lock);

	return 0;
}

static int fakelb_hw_virtual_rx_len(struct ieee802154_hw *hw)
{
	struct virtual_wpan_buf *vbuf;
	int len = 0;

	list_for_each_entry(vbuf, &fakelb_vbufs, list_vbuf) {
		if (vbuf->hw == hw) {
			len = vbuf->len;
			break;
		}
	}
	dev_dbg(hw->parent, "%s: return len=%d\n", __func__, len);

	return len;
}

static int fakelb_hw_virtual_rx(struct ieee802154_hw *hw, uint8_t *buf, size_t len)
{
	struct virtual_wpan_buf *vbuf;

	list_for_each_entry(vbuf, &fakelb_vbufs, list_vbuf) {
		if (vbuf->hw == hw) {
			memcpy(buf, vbuf->buf, len);

			list_del(&vbuf->list_vbuf);
			devm_kfree(hw->parent, vbuf);

			break;
		}
	}

	return 0;
}

static int fakelb_hw_rx(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *skb;
	int len;

	dev_dbg(hw->parent, "%s\n", __func__);
	skb = dev_alloc_skb(IEEE802154_MTU);
	if (skb) {
		len = fakelb_hw_virtual_rx_len(hw);
		fakelb_hw_virtual_rx(hw, skb_put(skb, len), len);

		ieee802154_rx_irqsafe(hw, skb, 0xcc);
		phy->flags &= ~(FAKELB_RXDONE);
#ifdef DEBUG
		print_hex_dump(KERN_DEBUG, "fakelb rx: ", DUMP_PREFIX_OFFSET, 16, 1,
							skb->data, skb->len, 0);
#endif
		return 0;
	}
	else {
		return -ENOMEM;
	}

}

static int fakelb_hw_virtual_tx(struct ieee802154_hw *hw, uint8_t *buf, size_t len)
{
	struct sx1278_phy *current_phy = hw->priv, *phy;
	struct virtual_wpan_buf *vbuf;

	list_for_each_entry(phy, &fakelb_ifup_phys, list_ifup) {
		if (current_phy == phy)
			continue;

		if (current_phy->page == phy->page &&
		    current_phy->channel == phy->channel) {
			vbuf = devm_kzalloc(phy->hw->parent, sizeof(struct virtual_wpan_buf), GFP_ATOMIC);
			vbuf->hw = phy->hw;
			memcpy(vbuf->buf, buf, len);
			vbuf->len = len;
			list_add(&vbuf->list_vbuf, &fakelb_vbufs);
#ifdef DEBUG
			print_hex_dump(KERN_DEBUG, "fakelb tx: ", DUMP_PREFIX_OFFSET, 16, 1,
								buf, len, 0);
#endif
			phy->flags |= FAKELB_RXDONE;
		}
	}

	current_phy->flags |= FAKELB_TXDONE;

	return 0;
}

static int fakelb_hw_tx_complete(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;
	struct sk_buff *skb = phy->txbuf;

	ieee802154_xmit_complete(hw, skb, false);

	spin_lock(&(phy->buf_lock));
	phy->txbuf = NULL;
	phy->flags &= ~(FAKELB_TXDONE);
	spin_unlock(&(phy->buf_lock));

	return 0;
}

static int fakelb_hw_xmit(struct ieee802154_hw *hw, struct sk_buff *skb)
{
	struct sx1278_phy *phy = hw->priv;
	int ret;

	read_lock_bh(&fakelb_ifup_phys_lock);
	WARN_ON(phy->suspended);
	read_unlock_bh(&fakelb_ifup_phys_lock);

	spin_lock(&(phy->buf_lock));
	if (phy->txbuf == NULL) {
		phy->txbuf = skb;
		ret = 0;
	}
	else {
		ret = -EBUSY;
	}
	spin_unlock(&(phy->buf_lock));

	if (ret == 0)
		fakelb_hw_virtual_tx(hw, skb->data, skb->len);

	return ret;
}

static int fakelb_hw_start(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;

	write_lock_bh(&fakelb_ifup_phys_lock);
	phy->suspended = false;
	list_add(&phy->list_ifup, &fakelb_ifup_phys);
	add_timer(&(phy->timer));
	write_unlock_bh(&fakelb_ifup_phys_lock);

	return 0;
}

static void fakelb_hw_stop(struct ieee802154_hw *hw)
{
	struct sx1278_phy *phy = hw->priv;

	write_lock_bh(&fakelb_ifup_phys_lock);
	phy->suspended = true;
	del_timer(&(phy->timer));
	list_del(&phy->list_ifup);
	write_unlock_bh(&fakelb_ifup_phys_lock);
}

static int
fakelb_set_promiscuous_mode(struct ieee802154_hw *hw, const bool on)
{
	return 0;
}

static void fakelb_timer_irqwork(struct work_struct *work)
{
	struct sx1278_phy *phy;
	u8 flags;

	phy = container_of(work, struct sx1278_phy, irqwork);
	flags = phy->flags;

	if (flags & FAKELB_RXDONE)
		fakelb_hw_rx(phy->hw);
	if (flags & FAKELB_TXDONE)
		fakelb_hw_tx_complete(phy->hw);

	if (!phy->suspended) {
		phy->timer.expires = jiffies + HZ * 20 / 1000;
		add_timer(&(phy->timer));
	}

	return;
}

static void fakelb_timer_isr(unsigned long arg)
{
	struct sx1278_phy *phy = (struct sx1278_phy *)arg;

	schedule_work(&(phy->irqwork));
}

static const struct ieee802154_ops fakelb_ops = {
	.owner = THIS_MODULE,
	.xmit_async = fakelb_hw_xmit,
	.ed = fakelb_hw_ed,
	.set_channel = fakelb_hw_channel,
	.start = fakelb_hw_start,
	.stop = fakelb_hw_stop,
	.set_promiscuous_mode = fakelb_set_promiscuous_mode,
};

struct rf_frq {
	uint32_t carrier;
	uint32_t bandwidth;
	uint8_t ch_min;
	uint8_t ch_max;
};

uint32_t
fakelb_hw_channel_mask(struct ieee802154_hw *hw)
{
	struct rf_frq rf;
	uint32_t mask;

	//sx127X_ieee_get_rf_config(hw, &rf);
	rf.ch_max = 11;
	rf.ch_min = 11;

	mask = ((uint32_t)(1 << (rf.ch_max + 1)) - (uint32_t)(1 << rf.ch_min));

	return mask;
}

static int sx1278_add_one(struct spi_device *spi)
{
	struct ieee802154_hw *hw;
	struct sx1278_phy *phy;
	int err;

	hw = ieee802154_alloc_hw(sizeof(*phy), &fakelb_ops);
	if (!hw) {
		dev_dbg(&(spi->dev), "not enough memory\n");
		return -ENOMEM;
	}

	phy = hw->priv;
	phy->hw = hw;
	phy->intf = spi;

	/* Set the SPI device's driver data for later usage. */
	spi_set_drvdata(spi, phy);

	/* Define channels could be used. */
	hw->phy->supported.channels[0] = fakelb_hw_channel_mask(hw);
	/* fake phy channel 11 as default */
	hw->phy->current_channel = 11;
	phy->channel = hw->phy->current_channel;

	ieee802154_random_extended_addr(&hw->phy->perm_extended_addr);
	hw->flags = IEEE802154_HW_TX_OMIT_CKSUM
			| IEEE802154_HW_RX_OMIT_CKSUM
			| IEEE802154_HW_PROMISCUOUS;
	hw->parent = &(spi->dev);

	err = ieee802154_register_hw(hw);
	if (err)
		goto err_reg;

	INIT_WORK(&(phy->irqwork), fakelb_timer_irqwork);

	init_timer(&(phy->timer));
	phy->timer.expires = jiffies + HZ;
	phy->timer.function = fakelb_timer_isr;
	phy->timer.data = (unsigned long)phy;

	spin_lock_init(&(phy->buf_lock));

	return 0;

err_reg:
	ieee802154_free_hw(phy->hw);
	dev_err(&(spi->dev), "register as IEEE 802.15.4 device failed\n");
	return err;
}

static void sx1278_del(struct spi_device *spi)
{
	struct sx1278_phy *phy = spi_get_drvdata(spi);

	if (phy == NULL)
		return;

	del_timer(&(phy->timer));
	flush_work(&(phy->irqwork));

	ieee802154_unregister_hw(phy->hw);
	ieee802154_free_hw(phy->hw);
}

/* The compatible chip array. */
#ifdef CONFIG_OF
static const struct of_device_id sx1278_dt_ids[] = {
	{ .compatible = "semtech,sx1276" },
	{ .compatible = "semtech,sx1277" },
	{ .compatible = "semtech,sx1278" },
	{ .compatible = "semtech,sx1279" },
	{ .compatible = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(of, sx1278_dt_ids);
#endif

/* The compatible ACPI device array. */
#ifdef CONFIG_ACPI
#define SX1278_ACPI_DUMMY	1
static const struct acpi_device_id sx1278_acpi_ids[] = {
	{ .id = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(acpi, sx1278_acpi_ids);

/* The callback function of ACPI probes SX1278 SPI. */
static void sx1278_probe_acpi(struct spi_device *spi) {
	const struct acpi_device_id *id;

	if (!has_acpi_companion(&(spi->dev)))
		return;

	id = acpi_match_device(sx1278_acpi_ids, &(spi->dev));
	if (WARN_ON(!id))
		return;

	if (id->driver_data == SX1278_ACPI_DUMMY)
		dev_warn(&(spi->dev),
			"Do not use this driver in produciton systems.\n");
}
#else
static void sx1278_probe_acpi(struct spi_device *spi) {};
#endif

/* The compatible SPI device id array. */
static const struct spi_device_id spi_ids[] = {
	{ .name = "sx1278" },
	{},
};
MODULE_DEVICE_TABLE(spi, spi_ids);

static int sx1278_spi_probe(struct spi_device *spi)
{
	int err;

	err = sx1278_add_one(spi);
	if (err < 0)
		goto err_slave;

	dev_info(&(spi->dev), "added a fake ieee802154 hardware devices\n");

	return 0;

err_slave:
	dev_err(&(spi->dev), "no SX1278 compatible device\n");
	sx1278_del(spi);
	return err;
}

static int sx1278_spi_remove(struct spi_device *spi)
{
	sx1278_del(spi);

	return 0;
}

#define __DRIVER_NAME	"sx1278"

/* The SPI driver which acts as a protocol driver in this kernel module. */
static struct spi_driver sx1278_spi_driver = {
	.driver = {
		.name = __DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sx1278_dt_ids,
#endif
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(sx1278_acpi_ids),
#endif
	},
	.probe = sx1278_spi_probe,
	.remove = sx1278_spi_remove,
	.id_table = spi_ids,
};

/* Register SX1278 kernel module. */
module_spi_driver(sx1278_spi_driver);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRa device SX1278 driver with SPI interface");
MODULE_LICENSE("Dual BSD/GPL");

/*-
 * Copyright (c) 2018 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/lora.h>

#define	FAKELR_DRIVER_NAME	"fakelr"

static LIST_HEAD(fakelr_hws);
static DEFINE_MUTEX(fakelr_hws_lock);

struct fakelr_phy {
	struct lora_hw *hw;
	struct list_head entry;
};

static int
fakelr_start(struct lora_hw *hw)
{
	dev_dbg(hw->parent, "%s\n", __func__);
	return 0;
}

static void
fakelr_stop(struct lora_hw *hw)
{
	dev_dbg(hw->parent, "%s\n", __func__);
}

static int
fakelr_xmit_async(struct lora_hw *hw, struct sk_buff *skb)
{
	dev_dbg(hw->parent, "%s\n", __func__);
	print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 16, 8, skb->data, skb->len, true);
	lora_xmit_complete(hw, skb);

	return 0;
}

static int
fakelr_set_txpower(struct lora_hw *hw, s32 pwr)
{
	dev_dbg(hw->parent, "%s: pwr=%d\n", __func__, pwr);
	return 0;
}

static int
fakelr_set_frq(struct lora_hw *hw, u32 frq)
{
	dev_dbg(hw->parent, "%s: frq=%ud\n", __func__, frq);
	return 0;
}

static int
fakelr_set_bw(struct lora_hw *hw, u32 bw)
{
	dev_dbg(hw->parent, "%s: bw=%ud\n", __func__, bw);
	return 0;
}

static int
fakelr_set_mod(struct lora_hw *hw, u8 mod)
{
	dev_dbg(hw->parent, "%s: mod=%ud\n", __func__, mod);
	return 0;
}

static int
fakelr_set_sf(struct lora_hw *hw, u8 sf)
{
	dev_dbg(hw->parent, "%s: sf=%ud\n", __func__, sf);
	return 0;
}

static int
fakelr_start_rx_window(struct lora_hw *hw, u32 t)
{
	dev_dbg(hw->parent, "%s: t=%ud\n", __func__, t);
	return 0;
}

static int
fakelr_set_state(struct lora_hw *hw, u8 state)
{
	dev_dbg(hw->parent, "%s: state=%ud\n", __func__, state);
	return 0;
}

static struct lora_operations lr_ops = {
	.start = fakelr_start,
	.stop = fakelr_stop,
	.xmit_async = fakelr_xmit_async,
	.set_txpower = fakelr_set_txpower,
	.set_frq = fakelr_set_frq,
	.set_bw = fakelr_set_bw,
	.set_mod = fakelr_set_mod,
	.set_sf = fakelr_set_sf,
	.start_rx_window = fakelr_start_rx_window,
	.set_state = fakelr_set_state,
};

struct fakelr_phy *
fakelr_add_one(struct device *dev)
{
	struct lora_hw *hw;
	struct fakelr_phy *phy;
	__le32 addr = cpu_to_le32(0x01020304);
	int err;

	/* Allocate a LoRa hardware */
	hw = lora_alloc_hw(sizeof(struct fakelr_phy), &lr_ops);
	if (!hw) {
		dev_err(dev, "Failed to allocate a LoRa device\n");
		phy = NULL;
		goto fakelr_add_one_end;
	}
	hw->parent = dev;
	phy = (struct fakelr_phy *)hw->priv;
	phy->hw = hw;

	lora_set_devaddr(hw, addr);

	/* Register the LoRa hardware */
	err = lora_register_hw(hw);
	if (err) {
		dev_err(dev, "Failed to register the LoRa hardware\n");
		lora_free_hw(hw);
		phy = NULL;
	}

fakelr_add_one_end:
	return phy;
}

void
fakelr_del(struct fakelr_phy *phy)
{
	struct lora_hw *hw = phy->hw;

	dev_info(hw->parent, "unregister a lora hardware\n");
	lora_unregister_hw(hw);
	lora_free_hw(hw);
}

static int
fakelr_probe(struct platform_device *pdev)
{
	struct fakelr_phy *phy;

	phy = fakelr_add_one(&pdev->dev);
	if (phy) {
		mutex_lock(&fakelr_hws_lock);
		list_add_tail(&phy->entry, &fakelr_hws);
		mutex_unlock(&fakelr_hws_lock);
	}

	return 0;
}

static int
fakelr_remove(struct platform_device *pdev)
{
	struct fakelr_phy *phy, *tmp;

	mutex_lock(&fakelr_hws_lock);
	list_for_each_entry_safe(phy, tmp, &fakelr_hws, entry) {
		list_del(&phy->entry);
		fakelr_del(phy);
	}
	mutex_unlock(&fakelr_hws_lock);

	return 0;
}

static struct platform_device *fakelr_dev;

static struct platform_driver fakelr_driver = {
	.probe = fakelr_probe,
	.remove = fakelr_remove,
	.driver = {
		.name = FAKELR_DRIVER_NAME,
	},
};

static int
fakelr_init(void)
{
	pr_info("%s: driver inserted", FAKELR_DRIVER_NAME);

	fakelr_dev = platform_device_register_simple(
		     FAKELR_DRIVER_NAME, -1, NULL, 0);

	return platform_driver_register(&fakelr_driver);
}

static void
fakelr_exit(void)
{
	pr_info("%s: driver removed\n", FAKELR_DRIVER_NAME);

	platform_driver_unregister(&fakelr_driver);
	platform_device_unregister(fakelr_dev);
}

module_init(fakelr_init);
module_exit(fakelr_exit);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("Fake LoRa device driver");
MODULE_LICENSE("Dual BSD/GPL");

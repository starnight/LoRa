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
#include <linux/device.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <net/rtnetlink.h>

#include <linux/lora.h>
#include "lorawan.h"

#define	PHY_NAME		"lora"

static struct class *lrw_sys_class;

static void
lora_if_setup(struct net_device *ndev)
{
	ndev->addr_len = LRW_DEVADDR_LEN;
	memset(ndev->broadcast, 0xFF, ndev->addr_len);
	ndev->type = ARPHRD_LORAWAN;

	ndev->hard_header_len = LRW_MHDR_LEN + LRW_FHDR_MAX_LEN + LRW_FPORT_LEN;
	ndev->needed_tailroom = LRW_MIC_LEN;

	// TODO: M should be a dynamic value defined by Regional Parameters
	//ndev->mtu = M - ndev->hard_header_len;
	ndev->mtu = 20;
}

/**
 * lora_alloc_hw - Allocate a memory space for the LoRa device
 * @priv_data_len:	the private data size
 * @lora_operations:	the implemented operations of the LoRa device
 *
 * Return:		address of the LoRa device or NULL for failed
 */
struct lora_hw *
lora_alloc_hw(size_t priv_data_len, struct lora_operations *ops)
{
	struct net_device *ndev;
	struct lrw_struct *lrw_st;
	int ret;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	if (WARN_ON(!ops || !ops->start || !ops->stop || !ops->xmit_async ||
		    !ops->set_txpower || !ops->set_frq || !ops->set_bw ||
		    !ops->set_mod || !ops->set_sf || !ops->start_rx_window ||
		    !ops->set_state))
		return NULL;

	/* In memory it'll be like this:
	 *
	 * +-----------------------+
	 * | struct net_device     |
	 * +-----------------------+
	 * | struct lrw_struct     |
	 * +-----------------------+
	 * | driver's private data |
	 * +-----------------------+
	 */
	ndev = alloc_netdev(sizeof(struct lrw_struct) + priv_data_len,
			    PHY_NAME"%d", NET_NAME_ENUM, lora_if_setup);
	if (!ndev)
		return ERR_PTR(-ENOMEM);
	ret = dev_alloc_name(ndev, ndev->name);
	if (ret < 0)
		goto lora_alloc_hw_err;

	lrw_st = (struct lrw_struct *)netdev_priv(ndev);
	lrw_st->ndev = ndev;

	lrw_st->state = LORA_STOP;
	lrw_st->ops = ops;
	lrw_st->hw.priv = (void *) lrw_st + sizeof(struct lrw_struct);

	//SET_NETDEV_DEV(ndev, &lrw_st->dev);
	//dev_net_set(ndev, &lrw_st->net);
	ndev->flags |= IFF_NOARP;
	ndev->features |= NETIF_F_HW_CSUM;

	return &lrw_st->hw;

lora_alloc_hw_err:
	free_netdev(ndev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(lora_alloc_hw);

/**
 * lora_free_hw - Free the LoRa device's memory resource
 * @hw:		the LoRa device going to be freed
 */
void
lora_free_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	free_netdev(lrw_st->ndev);
}
EXPORT_SYMBOL(lora_free_hw);

/**
 * lora_set_deveui - Set the LoRa device's DevEUI
 * @hw:		the LoRa device going to be set
 * @eui:	the global end-device ID in IEEE EUI64 address space
 */
void
lora_set_deveui(struct lora_hw *hw, __le64 eui)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	lrw_st->dev_eui = eui;
}
EXPORT_SYMBOL(lora_set_deveui);

/**
 * lora_get_deveui - Get the LoRa device's DevEUI
 * @hw:		the LoRa device going to be got from
 *
 * Return:	the device's DevEUI in IEEE EUI64 address space
 */
__le64
lora_get_deveui(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	return lrw_st->dev_eui;
}
EXPORT_SYMBOL(lora_get_deveui);

/**
 * lora_set_appeui - Set the LoRa device's AppEUI
 * @hw:		the LoRa device going to be set
 * @eui:	the global end-device ID in IEEE EUI64 address space
 */
void
lora_set_appeui(struct lora_hw *hw, __le64 eui)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	lrw_st->app_eui = eui;
}
EXPORT_SYMBOL(lora_set_appeui);

/**
 * lora_get_appeui - Get the LoRa device's AppEUI
 * @hw:		the LoRa device going to be got from
 *
 * Return:	the device's AppEUI in IEEE EUI64 address space
 */
__le64
lora_get_appeui(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	return lrw_st->app_eui;
}
EXPORT_SYMBOL(lora_get_appeui);

/**
 * lora_set_devaddr - Set the LoRa device's address
 * @hw:		the LoRa device going to be set
 * @devaddr:	the device address
 */
void
lora_set_devaddr(struct lora_hw *hw, __le32 devaddr)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	netdev_dbg(lrw_st->ndev, "%s: devaddr=%X\n",
		   __func__, le32_to_cpu(devaddr));
	lrw_st->devaddr = devaddr;
}
EXPORT_SYMBOL(lora_set_devaddr);

/**
 * lora_get_devaddr - Get the LoRa device's address
 * @hw:		the LoRa device going to be got from
 *
 * Return:	the device address
 */
__le32
lora_get_devaddr(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st;

	lrw_st = container_of(hw, struct lrw_struct, hw);
	return lrw_st->devaddr;
}
EXPORT_SYMBOL(lora_get_devaddr);

/**
 * lrw_add_hw - Add a LoRaWAN hardware as a network device
 * @lrw_st:	the LoRa device going to be added
 *
 * Return:	0 / other number for success / failed
 */
int
lrw_add_hw(struct lrw_struct *lrw_st)
{
	struct net_device *ndev = lrw_st->ndev;
	__be32 be_addr;
	int ret;

	netdev_dbg(ndev, "%s\n", __func__);

	lrw_st->fcnt_up = 0;
	lrw_st->fcnt_down = 0;
	lrw_st->_cur_ss = NULL;

	mutex_init(&lrw_st->ss_list_lock);
	INIT_LIST_HEAD(&lrw_st->ss_list);

	tasklet_init(&lrw_st->xmit_task, lrw_xmit, (unsigned long) lrw_st);
	INIT_WORK(&lrw_st->rx_work, lrw_rx_work);

	be_addr = le32_to_be32(lrw_st->devaddr);
	memcpy(ndev->perm_addr, &be_addr, ndev->addr_len);
	memcpy(ndev->dev_addr, ndev->perm_addr, ndev->addr_len);

	write_pnet(&lrw_st->_net, &init_net);
	ret = register_netdev(ndev);

	return ret;
}

/**
 * lrw_remove_hw - Remove a LoRaWAN hardware from a network device
 * @lrw_st:	the LoRa device going to be removed
 */
void
lrw_remove_hw(struct lrw_struct *lrw_st)
{
	unregister_netdev(lrw_st->ndev);
	tasklet_kill(&lrw_st->xmit_task);
}

int
lrw_set_hw_state(struct lrw_struct *lrw_st, void __user *arg)
{
	int ret = 0;
	u8 state;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	ret = copy_from_user(&state, arg, 1);
	if (ret) {
		ret = -EACCES;
		goto lrw_set_hw_state_end;
	}

	switch (state) {
	case LORA_START:
		if (lrw_st->state == LORA_STOP)
			lora_start_hw(lrw_st);
		break;
	case LORA_STOP:
		if (lrw_st->state != LORA_STOP)
			lora_stop_hw(lrw_st);
		break;
	default:
		ret = -ENOTSUPP;
	}

lrw_set_hw_state_end:
	return ret;
}

bool
ready2write(struct lrw_struct *lrw_st)
{
	bool status = false;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);
	if ((!lrw_st->_cur_ss) && (lrw_st->state == LORA_STATE_IDLE))
		status = true;

	return status;
}

bool
ready2read(struct lrw_struct *lrw_st)
{
	bool status = false;
	struct lrw_session *ss;

	if (!list_empty(&lrw_st->ss_list) && (lrw_st->state != LORA_STOP)) {
		ss = list_first_entry(&lrw_st->ss_list,
				      struct lrw_session,
				      entry);
		if (ss->state == LRW_RXRECEIVED_SS)
			status = true;
	}

	return status;
}

static int
lrw_if_up(struct net_device *ndev)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	int ret = -EBUSY;

	netdev_dbg(ndev, "%s\n", __func__);

	if (lrw_st->state == LORA_STOP) {
		ret = lora_start_hw(lrw_st);
		netif_start_queue(ndev);
	}

	return ret;
}

static int
lrw_if_down(struct net_device *ndev)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);

	netdev_dbg(ndev, "%s\n", __func__);

	if (lrw_st->state != LORA_STOP) {
		netif_stop_queue(ndev);
		lora_stop_hw(lrw_st);
	}

	return 0;
}

netdev_tx_t
lrw_if_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	struct lrw_session *ss;
	netdev_tx_t ret = NETDEV_TX_OK;

	pr_debug("%s: xmit skb (size=%u)\n", LORAWAN_MODULE_NAME, skb->len);

	ss = lrw_alloc_ss(lrw_st);
	if (!ss)
		return NETDEV_TX_BUSY;

	mutex_lock(&lrw_st->ss_list_lock);
	if (ready2write(lrw_st)) {
		list_add_tail(&ss->entry, &lrw_st->ss_list);
		lrw_st->state = LORA_STATE_TX;
		lrw_st->_cur_ss = ss;
		lrw_st->fcnt_up += 1;
		ss->fcnt_up = lrw_st->fcnt_up;
		ss->fcnt_down = lrw_st->fcnt_down;
	}
	else
		ret = NETDEV_TX_BUSY;
	mutex_unlock(&lrw_st->ss_list_lock);

	if (ret == NETDEV_TX_OK) {
		pr_debug("%s: write a new skb\n", LORAWAN_MODULE_NAME);
		ss->state = LRW_INIT_SS;
		ss->tx_skb = skb;
		lrw_prepare_tx_frame(ss);
		tasklet_schedule(&lrw_st->xmit_task);
	}
	else
		lrw_free_ss(ss);

	return ret;
}


inline int
lrw_if_get_addr(struct lrw_struct *lrw_st, struct sockaddr_lorawan *addr)
{
	int ret = 0;

	switch (addr->addr_in.addr_type) {
	case LRW_ADDR_DEVADDR:
		addr->addr_in.devaddr = le32_to_cpu(lrw_st->devaddr);
		break;
	case LRW_ADDR_DEVEUI:
		addr->addr_in.dev_eui = le64_to_cpu(lrw_st->dev_eui);
		break;
	case LRW_ADDR_APPEUI:
		addr->addr_in.app_eui = le64_to_cpu(lrw_st->app_eui);
		break;
	default:
		ret = -ENOTSUPP;
	}

	return ret;
}

inline int
lrw_if_set_addr(struct lrw_struct *lrw_st, struct sockaddr_lorawan *addr)
{
	struct lora_hw *hw = &lrw_st->hw;
	int ret = 0;

	if (netif_running(lrw_st->ndev))
		return -EBUSY;

	switch (addr->addr_in.addr_type) {
	case LRW_ADDR_DEVADDR:
		lora_set_devaddr(hw, cpu_to_le32(addr->addr_in.devaddr));
		break;
	case LRW_ADDR_DEVEUI:
		lora_set_deveui(hw, cpu_to_le32(addr->addr_in.dev_eui));
		break;
	case LRW_ADDR_APPEUI:
		lora_set_appeui(hw, cpu_to_le32(addr->addr_in.app_eui));
		break;
	default:
		ret = -ENOTSUPP;
	}

	return ret;
}

inline void
swap_bytes(u8 *dst, u8 *src, size_t l)
{
	/* Human reading is big-endian, but LoRaWAN is little-endian */
	unsigned int i;
	for (i = 0; i < l; i++)
		dst[i] = src[l - i - 1];
}

int
lora_set_key(struct lora_hw *hw, u8 type, u8 *key, size_t key_len)
{
	struct lrw_struct *lrw_st;
	int ret = 0;

	lrw_st = container_of(hw, struct lrw_struct, hw);

	netdev_dbg(lrw_st->ndev, "%s: type=%d\n", __func__, type);
	if (lrw_st->state != LORA_STOP)
		return -EINVAL;

	print_hex_dump(KERN_DEBUG, "", DUMP_PREFIX_OFFSET, 16, 1, key, key_len, true);
	switch (type) {
	case LORA_APPKEY:
		swap_bytes(lrw_st->appkey, key, key_len);
		break;
	case LORA_NWKSKEY:
		swap_bytes(lrw_st->nwkskey, key, key_len);
		break;
	case LORA_APPSKEY:
		swap_bytes(lrw_st->appskey, key, key_len);
		break;
	default:
		ret = -ENOTSUPP;
	}

	return ret;
}
EXPORT_SYMBOL(lora_set_key);

int
lora_get_key(struct lora_hw *hw, u8 type, u8 *key, size_t key_len)
{
	struct lrw_struct *lrw_st;
	int ret = 0;

	lrw_st = container_of(hw, struct lrw_struct, hw);

	netdev_dbg(lrw_st->ndev, "%s: type=%d\n", __func__, type);
	switch (type) {
	case LORA_APPKEY:
		swap_bytes(key, lrw_st->appkey, key_len);
		break;
	case LORA_NWKSKEY:
		swap_bytes(key, lrw_st->nwkskey, key_len);
		break;
	case LORA_APPSKEY:
		swap_bytes(key, lrw_st->appskey, key_len);
		break;
	default:
		ret = -ENOTSUPP;
	}

	return ret;
}

static int
lrw_if_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	struct sockaddr_lorawan *addr;
	struct lora_key lr_key;
	s32 txpower;
	int ret = 0;

	netdev_dbg(ndev, "%s: ioctl file (cmd=0x%X)\n", __func__, cmd);

	/* I/O control by each command */
	switch (cmd) {
	/* Set & get the DevAddr, DevEUI and AppEUI */
	case SIOCSIFADDR:
		addr = (struct sockaddr_lorawan *)&ifr->ifr_addr;
		ret = lrw_if_set_addr(lrw_st, addr);
		break;
	case SIOCGIFADDR:
		addr = (struct sockaddr_lorawan *)&ifr->ifr_addr;
		ret = lrw_if_get_addr(lrw_st, addr);
		break;
	/* Set & get the key */
	case SIOCSLRWKEY:
		if (copy_from_user(&lr_key, ifr->ifr_data, sizeof(lr_key))) {
			ret = -EFAULT;
			break;
		}
		ret = lora_set_key(&lrw_st->hw,
				   lr_key.type, lr_key.key, LORA_KEY_LEN);
		break;
	case SIOCGLRWKEY:
		if (copy_from_user(&lr_key, ifr->ifr_data, sizeof(lr_key))) {
			ret = -EFAULT;
			break;
		}
		ret = lora_get_key(&lrw_st->hw,
				   lr_key.type, lr_key.key, LORA_KEY_LEN);
		if (ret)
			break;

		if (copy_to_user(ifr->ifr_data, &lr_key, sizeof(lr_key)))
			ret = -EFAULT;
		break;
	/* Set & get the PA power */
	case SIOCSLRWTXPWR:
		get_user(txpower, (s32 __user *)ifr->ifr_data);
		ret = lrw_st->ops->set_txpower(&lrw_st->hw, txpower);
		break;
	case SIOCGLRWTXPWR:
		txpower = lrw_st->hw.transmit_power;
		ret = put_user(txpower, (s32 __user *)ifr->ifr_data);
		break;
	default:
		ret = -ENOTSUPP;
	}

	return ret;
}

static int
lrw_if_set_mac(struct net_device *ndev, void *p)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	struct sockaddr *addr = p;
	__be32 *be_addr = (__be32 *)addr->sa_data;

	netdev_dbg(ndev, "%s: AF_TYPE:%d set mac address %X\n",
		   __func__, addr->sa_family, be32_to_cpu(*be_addr));

	if (netif_running(ndev))
		return -EBUSY;

	lora_set_devaddr(&lrw_st->hw, be32_to_le32(*be_addr));
	memcpy(ndev->dev_addr, be_addr, ndev->addr_len);

	return 0;
}

static const struct net_device_ops lrw_if_ops = {
	.ndo_open = lrw_if_up,
	.ndo_stop = lrw_if_down,
	.ndo_start_xmit = lrw_if_start_xmit,
	.ndo_do_ioctl = lrw_if_ioctl,
	.ndo_set_mac_address = lrw_if_set_mac,
};

/**
 * lora_register_hw - Register there is a kind of LoRa device
 * @hw:		LoRa device going to be registered
 *
 * Return:	0 / negative number for success / error number
 */
int
lora_register_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st = container_of(hw, struct lrw_struct, hw);
	int ret;

	pr_debug("%s: %s\n", LORAWAN_MODULE_NAME, __func__);

	device_initialize(&lrw_st->dev);
	dev_set_name(&lrw_st->dev, netdev_name(lrw_st->ndev));
	lrw_st->dev.class = lrw_sys_class;
	lrw_st->dev.platform_data = lrw_st;

	ret = device_add(&lrw_st->dev);
	if (ret)
		goto lora_register_hw_end;

	/* Add a LoRa device node as a network device */
	lrw_st->ndev->netdev_ops = &lrw_if_ops;
	ret = lrw_add_hw(lrw_st);

lora_register_hw_end:
	return ret;
}
EXPORT_SYMBOL(lora_register_hw);

/**
 * lora_unregister_hw - Unregister the LoRa driver
 * @hw:		LoRa device going to be unregistered
 */
void
lora_unregister_hw(struct lora_hw *hw)
{
	struct lrw_struct *lrw_st = container_of(hw, struct lrw_struct, hw);

	pr_info("%s: unregister %s\n",
		LORAWAN_MODULE_NAME, dev_name(&lrw_st->dev));

	/* Stop and remove the LoRaWAM hardware from system */
	if (lrw_st->state != LORA_STOP)
		lora_stop_hw(lrw_st);
	device_del(&lrw_st->dev);
	lrw_remove_hw(lrw_st);

	return;
}
EXPORT_SYMBOL(lora_unregister_hw);

static int __init
lrw_init(void)
{
	int err;

	pr_info("%s: module inserted\n", LORAWAN_MODULE_NAME);

	/* Create device class */
	lrw_sys_class = class_create(THIS_MODULE, LORAWAN_MODULE_NAME);
	if (IS_ERR(lrw_sys_class)) {
		pr_err("%s: Failed to create a class of LoRaWAN\n",
		       LORAWAN_MODULE_NAME);
		err = PTR_ERR(lrw_sys_class);
		goto lrw_init_end;
	}

	pr_debug("%s: class created\n", LORAWAN_MODULE_NAME);

	/* Initial LoRaWAN socket API */
	err = lrw_sock_init();

lrw_init_end:
	return err;
}

static void __exit
lrw_exit(void)
{
	/* Release LoRaWAN socket API */
	lrw_sock_exit();
	/* Delete device class */
	class_destroy(lrw_sys_class);
	pr_info("%s: module removed\n", LORAWAN_MODULE_NAME);

	return;
}

module_init(lrw_init);
module_exit(lrw_exit);

MODULE_AUTHOR("Jian-Hong Pan, <starnight@g.ncu.edu.tw>");
MODULE_DESCRIPTION("LoRaWAN kernel module");
MODULE_LICENSE("Dual BSD/GPL");

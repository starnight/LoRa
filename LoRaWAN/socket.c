#include <linux/list.h>
#include <linux/net.h>
#include <linux/if_arp.h>
#include <linux/termios.h>	/* For TIOCOUTQ/INQ */
#include <net/sock.h>

#include "lorawan.h"

struct dgram_sock {
	struct sock sk;
	u32 src_devaddr;

	unsigned int bound:1;
	unsigned int connected:1;
	unsigned int want_ack:1;
	unsigned int secen:1;
	unsigned int secen_override:1;
	unsigned int seclevel:3;
	unsigned int seclevel_override:1;
};

static HLIST_HEAD(dgram_head);
static DEFINE_RWLOCK(dgram_lock);

inline struct dgram_sock *
dgram_sk(const struct sock *sk)
{
	return container_of(sk, struct dgram_sock, sk);
}

//inline void
//lrw_addr_from_sain(struct lrw_addr *a, struct lrw_addr_in *sa_in)
//{
//	memcpy(a->devaddr, sa_in->devaddr, LRW_DEVADDR_LEN);
//}

//inline void
//lrw_addr_to_sain(struct lrw_addr_in *sa_in, struct lrw_addr *a)
//{
//	memcpy(sa_in->devaddr, a->devaddr, LRW_DEVADDR_LEN);
//}

inline struct net_device *
lrw_get_dev_by_addr(struct net *net, u32 devaddr)
{
	struct net_device *ndev = NULL;
	__be32 be_addr = cpu_to_be32(devaddr);

	rcu_read_lock();
	ndev = dev_getbyhwaddr_rcu(net, ARPHRD_LORAWAN, (char *)&be_addr);
	if (ndev)
		dev_hold(ndev);
	rcu_read_unlock();

	return ndev;
}

inline struct lrw_mac_cb *
mac_cb(struct sk_buff *skb)
{
	return (struct lrw_mac_cb *)skb->cb;
}

static int
dgram_init(struct sock *sk)
{
	struct dgram_sock *ro = dgram_sk(sk);

	ro->want_ack = 1;
	return 0;
}

static void
dgram_close(struct sock *sk, long timeout)
{
	sk_common_release(sk);
}

static int
dgram_bind(struct sock *sk, struct sockaddr *uaddr, int len)
{
	struct sockaddr_lorawan *addr = (struct sockaddr_lorawan *)uaddr;
	struct dgram_sock *ro = dgram_sk(sk);
	struct net_device *ndev;
	int ret;

	lock_sock(sk);
	ro->bound = 0;

	ret = -EINVAL;
	if (len < sizeof(*addr))
		goto dgram_bind_end;

	if (addr->family != AF_LORAWAN)
		goto dgram_bind_end;

	if (addr->addr_in.addr_type != LRW_ADDR_DEVADDR)
		goto dgram_bind_end;

	ndev = lrw_get_dev_by_addr(sock_net(sk), addr->addr_in.devaddr);
	if (!ndev) {
		ret = -ENODEV;
		goto dgram_bind_end;
	}

	if (ndev->type != ARPHRD_LORAWAN) {
		ret = -ENODEV;
		goto dgram_bind_end;
	}

	ro->src_devaddr = addr->addr_in.devaddr;
	ro->bound = 1;
	ret = 0;

dgram_bind_end:
	release_sock(sk);
	return ret;
}

inline int
lrw_dev_hard_header(struct sk_buff *skb, struct net_device *ndev,
		    const u32 src_devaddr, size_t len)
{
	/* TODO: Prepare the LoRaWAN sending header here */
	return 0;
}

static int
dgram_sendmsg(struct sock *sk, struct msghdr *msg, size_t size)
{
	struct dgram_sock *ro = dgram_sk(sk);
	struct net_device *ndev;
	struct sk_buff *skb;
	size_t hlen;
	size_t mtu;
	size_t tlen;
	int ret;

	if (msg->msg_flags & MSG_OOB) {
		pr_debug("msg->msg_flags = 0x%x\n", msg->msg_flags);
		return -EOPNOTSUPP;
	}

	if (!ro->connected && !msg->msg_name)
		return -EDESTADDRREQ;
	else if (ro->connected && msg->msg_name)
		return -EISCONN;

	if (!ro->bound)
		ndev = dev_getfirstbyhwtype(sock_net(sk), ARPHRD_LORAWAN);
	else
		ndev = lrw_get_dev_by_addr(sock_net(sk), ro->src_devaddr);

	if (!ndev) {
		pr_debug("no dev\n");
		ret = -ENXIO;
		goto dgram_sendmsg_end;
	}

	/* TODO: MTU should be the regional defined */
	mtu = LORAWAN_MTU;
	if (size > mtu){
		pr_debug("size = %zu, mtu = %zu\n", size, mtu);
		ret = -EMSGSIZE;
		goto dgram_sendmsg_end;
	}

	hlen = LL_RESERVED_SPACE(ndev);
	tlen = ndev->needed_tailroom;
	skb = sock_alloc_send_skb(sk, hlen + tlen + size,
				  msg->msg_flags & MSG_DONTWAIT,
				  &ret);

	if (!skb)
		goto dgram_sendmsg_no_skb;

	skb_reserve(skb, hlen);
	skb_reset_network_header(skb);
//	if (msg->msg_name) {
//		DECLARE_SOCKADDR(struct sockaddr_lorawan*,
//				 daddr, msg->msg_name);
//
//		lrw_addr_from_sa(&dst_addr, &daddr->addr_sa);
//	} else {
//		//
//		dst_addr = ro->dst_addr;
//	}

	ret = lrw_dev_hard_header(skb, ndev, ro->bound ? ro->src_devaddr : 0, size);
	if (ret < 0)
		goto dgram_sendmsg_no_skb;

	ret = memcpy_from_msg(skb_put(skb, size), msg, size);
	if (ret > 0)
		goto dgram_sendmsg_err_skb;

	skb->dev = ndev;
	skb->protocol = htons(ETH_P_LORAWAN);

	ret = dev_queue_xmit(skb);
	if (ret > 0)
		ret = net_xmit_errno(ret);

	dev_put(ndev);

	return ret ?: size;

dgram_sendmsg_err_skb:
	kfree_skb(skb);
dgram_sendmsg_no_skb:
	dev_put(ndev);

dgram_sendmsg_end:
	return ret;
}

static int
dgram_recvmsg(struct sock *sk, struct msghdr *msg, size_t len,
	      int noblock, int flags, int *addr_len)
{
	struct sk_buff *skb;
	size_t copied = 0;
	DECLARE_SOCKADDR(struct sockaddr_lorawan *, saddr, msg->msg_name);
	int err;

	skb = skb_recv_datagram(sk, flags, noblock, &err);
	if (!skb)
		goto dgram_recvmsg_end;

	copied = skb->len;
	if (len < copied) {
		msg->msg_flags |= MSG_TRUNC;
		copied = len;
	}

	err = skb_copy_datagram_msg(skb, 0, msg, copied);
	if (err)
		goto dgram_recvmsg_done;

	sock_recv_ts_and_drops(msg, sk, skb);
	if(saddr) {
		memset(saddr, 0, sizeof(*saddr));
		saddr->family = AF_LORAWAN;
		saddr->addr_in.devaddr = mac_cb(skb)->devaddr;
		*addr_len = sizeof(*saddr);
	}

	if (flags & MSG_TRUNC)
		copied = skb->len;

dgram_recvmsg_done:
	skb_free_datagram(sk, skb);

dgram_recvmsg_end:
	if (err)
		return err;
	return copied;
}

static int
dgram_hash(struct sock *sk)
{
	write_lock_bh(&dgram_lock);
	sk_add_node(sk, &dgram_head);
	sock_prot_inuse_add(sock_net(sk), sk->sk_prot, 1);
	write_unlock_bh(&dgram_lock);

	return 0;
}

static void
dgram_unhash(struct sock *sk)
{
	write_lock_bh(&dgram_lock);
	if (sk_del_node_init(sk))
		sock_prot_inuse_add(sock_net(sk), sk->sk_prot, -1);
	write_unlock_bh(&dgram_lock);
}

static int
dgram_connect(struct sock *sk, struct sockaddr *uaddr, int len)
{
	struct dgram_sock *ro = dgram_sk(sk);

	/* Nodes of LoRaWAN send data to a gateway only, then data is received
	 * and transferred to servers with the gateway's policy.
	 * So, the destination address is not used by nodes.
	 */
	lock_sock(sk);
	ro->connected = 1;
	release_sock(sk);

	return 0;
}

static int
dgram_disconnect(struct sock *sk, int flags)
{
	struct dgram_sock *ro = dgram_sk(sk);

	lock_sock(sk);
	ro->connected = 0;
	release_sock(sk);

	return 0;
}

static int
dgram_ioctl(struct sock *sk, int cmd, unsigned long arg)
{
	struct sk_buff *skb;
	int amount;
	int err;

	switch (cmd) {
	case SIOCOUTQ:
		amount = sk_wmem_alloc_get(sk);
		err = put_user(amount, (int __user *)arg);
		break;
	case SIOCINQ:
		amount = 0;
		spin_lock_bh(&sk->sk_receive_queue.lock);
		skb = skb_peek(&sk->sk_receive_queue);
		if (skb) {
			/* We will only return the amount of this packet
			 * since that is all that will be read.
			 */
			amount = skb->len - 0;//lrw_hdr_length(skb);
		}
		spin_unlock_bh(&sk->sk_receive_queue.lock);
		err = put_user(amount, (int __user *)arg);
		break;
	default:
		err = -ENOIOCTLCMD;
	}

	return err;
}

static int
dgram_getsockopt(struct sock *sk, int level, int optname,
		 char __user *optval, int __user *optlen)
{
	int val, len;

	if (level != SOL_LORAWAN)
		return -EOPNOTSUPP;

	if (get_user(len, optlen))
		return -EFAULT;

	len = min_t(unsigned int, len, sizeof(int));

	switch (optname) {
	default:
		return -ENOPROTOOPT;
	}

	if (put_user(len, optlen))
		return -EFAULT;

	if (copy_to_user(optval, &val, len))
		return -EFAULT;

	return 0;
}

static int
dgram_setsockopt(struct sock *sk, int level, int optname,
		 char __user *optval, unsigned int optlen)
{
	int val;
	int err = 0;

	if (optlen < sizeof(int))
		return -EINVAL;

	if (get_user(val, (int __user *)optval))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	default:
		err = -ENOPROTOOPT;
		break;
	}

	release_sock(sk);

	return err;
}

static struct proto lrw_dgram_prot = {
	.name		= "LoRaWAN-MAC",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct dgram_sock),
	.init		= dgram_init,
	.close		= dgram_close,
	.bind		= dgram_bind,
	.sendmsg	= dgram_sendmsg,
	.recvmsg	= dgram_recvmsg,
	.hash		= dgram_hash,
	.unhash		= dgram_unhash,
	.connect	= dgram_connect,
	.disconnect	= dgram_disconnect,
	.ioctl		= dgram_ioctl,
	.getsockopt	= dgram_getsockopt,
	.setsockopt	= dgram_setsockopt,
};

static int
lrw_sock_release(struct socket *sock)
{
	struct sock *sk = sock->sk;

	if (sk) {
		sock->sk = NULL;
		sk->sk_prot->close(sk, 0);
	}

	return 0;
}

static int
lrw_sock_bind(struct socket *sock, struct sockaddr *uaddr, int addr_len)
{
	struct sock *sk = sock->sk;

	if (sk->sk_prot->bind)
		return sk->sk_prot->bind(sk, uaddr, addr_len);

	return sock_no_bind(sock, uaddr, addr_len);
}

static int
lrw_sock_connect(struct socket *sock, struct sockaddr *uaddr,
		 int addr_len, int flags)
{
	struct sock *sk = sock->sk;

	if (addr_len < sizeof(uaddr->sa_family))
		return -EINVAL;

	//if (uaddr->sa_family == AF_UNSPEC)
	//	return sk->sk_prot->disconnect(sk, flags);

	return sk->sk_prot->connect(sk, uaddr, addr_len);
}

static int
lrw_ndev_ioctl(struct sock *sk, struct ifreq __user *arg, unsigned int cmd)
{
	struct ifreq ifr;
	int ret = -ENOIOCTLCMD;
	struct net_device *ndev;

	pr_debug("lorawan: %s: cmd %ud\n", __func__, cmd);
	if (copy_from_user(&ifr, arg, sizeof(struct ifreq)))
		return -EFAULT;

	ifr.ifr_name[IFNAMSIZ-1] = 0;

	dev_load(sock_net(sk), ifr.ifr_name);
	ndev = dev_get_by_name(sock_net(sk), ifr.ifr_name);

	netdev_dbg(ndev, "%s: cmd %ud\n", __func__, cmd);
	if (!ndev)
		return -ENODEV;

	if (ndev->type == ARPHRD_LORAWAN && ndev->netdev_ops->ndo_do_ioctl)
		ret = ndev->netdev_ops->ndo_do_ioctl(ndev, &ifr, cmd);

	if (!ret && copy_to_user(arg, &ifr, sizeof(struct ifreq)))
		ret = -EFAULT;
	dev_put(ndev);

	return ret;
}

static int
lrw_sock_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	struct sock *sk = sock->sk;
	struct net_device *ndev = sk->sk_dst_cache->dev;

	netdev_dbg(ndev, "%s: cmd %ud\n", __func__, cmd);
	switch (cmd) {
	case SIOCGSTAMP:
		return sock_get_timestamp(sk, (struct timeval __user *)arg);
	case SIOCGSTAMPNS:
		return sock_get_timestampns(sk, (struct timespec __user *)arg);
	case SIOCOUTQ:
	case SIOCINQ:
		if (!sk->sk_prot->ioctl)
			return -ENOIOCTLCMD;
		return sk->sk_prot->ioctl(sk, cmd, arg);
	default:
		return lrw_ndev_ioctl(sk, (struct ifreq __user *)arg, cmd);
	}
}

static int
lrw_sock_sendmsg(struct socket *sock, struct msghdr *msg, size_t len)
{
	struct sock *sk = sock->sk;

	return sk->sk_prot->sendmsg(sk, msg, len);
}

static const struct proto_ops lrw_dgram_ops = {
	.family		= PF_LORAWAN,
	.owner		= THIS_MODULE,
	.release	= lrw_sock_release,
	.bind		= lrw_sock_bind,
	.connect	= lrw_sock_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= sock_no_getname,
	.poll		= datagram_poll,
	.ioctl		= lrw_sock_ioctl,
	.listen		= sock_no_listen,
	.shutdown	= sock_no_shutdown,
	.setsockopt	= sock_common_setsockopt,
	.getsockopt	= sock_common_getsockopt,
	.sendmsg	= lrw_sock_sendmsg,
	.recvmsg	= sock_common_recvmsg,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage,
};

static int
lorawan_creat(struct net *net, struct socket *sock, int protocol, int kern)
{
	struct sock *sk;
	int ret;

	if (!net_eq(net, &init_net))
		return -EAFNOSUPPORT;

	if (sock->type != SOCK_DGRAM)
		return -EAFNOSUPPORT;

	sk = sk_alloc(net, PF_LORAWAN, GFP_KERNEL, &lrw_dgram_prot, kern);
	if (!sk)
		return -ENOMEM;

	sock->ops = &lrw_dgram_ops;
	sock_init_data(sock, sk);
	sk->sk_family = PF_LORAWAN;
	sock_set_flag(sk, SOCK_ZAPPED);

	if (sk->sk_prot->hash) {
		ret = sk->sk_prot->hash(sk);
		if (ret) {
			sk_common_release(sk);
			goto lorawan_creat_end;
		}
	}

	if (sk->sk_prot->init) {
		ret = sk->sk_prot->init(sk);
		if (ret)
			sk_common_release(sk);
	}

lorawan_creat_end:
	return ret;
}

static const struct net_proto_family lorawan_family_ops = {
	.owner		= THIS_MODULE,
	.family		= PF_LORAWAN,
	.create		= lorawan_creat,
};

inline int
lrw_dgram_deliver(struct net_device *ndev, struct sk_buff *skb)
{
	struct lrw_struct *lrw_st = NETDEV_2_LRW(ndev);
	struct sock *sk;
	struct dgram_sock *ro;
	bool found = false;
	int ret = NET_RX_SUCCESS;

	read_lock(&dgram_lock);
	sk_for_each(sk, &dgram_head) {
		ro = dgram_sk(sk);
		if(cpu_to_le32(ro->src_devaddr) == lrw_st->devaddr) {
			found = true;
			break;
		}
	}
	read_unlock(&dgram_lock);

	if (!found)
		goto lrw_dgram_deliver_err;

	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb)
		return NET_RX_DROP;

	if (sock_queue_rcv_skb(sk, skb) < 0)
		goto lrw_dgram_deliver_err;

	return ret;

lrw_dgram_deliver_err:
	kfree_skb(skb);
	ret = NET_RX_DROP;
	return ret;
}

static int
lorawan_rcv(struct sk_buff *skb, struct net_device *ndev,
	    struct packet_type *pt, struct net_device *orig_ndev)
{
	if (!netif_running(ndev))
		goto lorawan_rcv_drop;

	if (!net_eq(dev_net(ndev), &init_net))
		goto lorawan_rcv_drop;

	if (ndev->type != ARPHRD_LORAWAN)
		goto lorawan_rcv_drop;

	if (skb->pkt_type != PACKET_OTHERHOST)
		return lrw_dgram_deliver(ndev, skb);

lorawan_rcv_drop:
	kfree_skb(skb);
	return NET_RX_DROP;
}

static struct packet_type lorawan_packet_type = {
	.type		= htons(ETH_P_LORAWAN),
	.func		= lorawan_rcv,
};

int
lrw_sock_init(void)
{
	int ret;

	ret = proto_register(&lrw_dgram_prot, 1);
	if(ret)
		goto lrw_sock_init_end;

	/* Tell SOCKET that we are alive */
	ret = sock_register(&lorawan_family_ops);
	if(ret)
		goto lrw_sock_init_err;

	dev_add_pack(&lorawan_packet_type);
	ret = 0;
	goto lrw_sock_init_end;

lrw_sock_init_err:
	proto_unregister(&lrw_dgram_prot);

lrw_sock_init_end:
	return 0;
}

void
lrw_sock_exit(void)
{
	dev_remove_pack(&lorawan_packet_type);
	sock_unregister(PF_LORAWAN);
	proto_unregister(&lrw_dgram_prot);
}

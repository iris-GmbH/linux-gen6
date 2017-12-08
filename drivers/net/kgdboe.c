/*
 * drivers/net/kgdboe.c
 *
 * A network interface for GDB.
 * Based upon 'gdbserial' by David Grothe <dave@gcom.com>
 * and Scott Foehner <sfoehner@engr.sgi.com>
 *
 * Maintainer: Jason Wessel <jason.wessel@windriver.com>
 *
 * 2004 (c) Amit S. Kale <amitkale@linsyssoft.com>
 * 2004-2005 (c) MontaVista Software, Inc.
 * 2005-2008 (c) Wind River Systems, Inc.
 *
 * Contributors at various stages not listed above:
 * San Mehat <nettwerk@biodome.org>, Robert Walsh <rjwalsh@durables.org>,
 * wangdi <wangdi@clusterfs.com>, Matt Mackall <mpm@selenic.com>,
 * Pavel Machek <pavel@suse.cz>, Jason Wessel <jason.wessel@windriver.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/kgdb.h>
#include <linux/netdevice.h>
#include <linux/netpoll.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/inetdevice.h>
#include <linux/inet.h>
#include <net/tcp.h>
#include <net/udp.h>
#include <net/ip6_checksum.h>


#include <asm/atomic.h>

#define IN_BUF_SIZE 512		/* power of 2, please */
#define OUT_BUF_SIZE 30		/* We don't want to send too big of a packet. */
#define MAX_CONFIG_LEN 256

static char in_buf[IN_BUF_SIZE], out_buf[OUT_BUF_SIZE];
static int in_head, in_tail, out_count;
static atomic_t in_count;
/* 0 = unconfigured, 1 = netpoll options parsed, 2 = fully configured. */
static int configured;
static struct kgdb_io local_kgdb_io_ops;

MODULE_DESCRIPTION("KGDB driver for network interfaces");
MODULE_LICENSE("GPL");
static char config[MAX_CONFIG_LEN];
static struct kparam_string kps = {
	.string = config,
	.maxlen = MAX_CONFIG_LEN,
};

struct netpoll_info_ex {
	unsigned long rx_flags;
	spinlock_t rx_lock;
                      
	struct sk_buff_head neigh_tx; /* list of neigh requests to reply to */
};

static struct netpoll_info_ex npinfo_ex;

static struct netpoll kgdb_np = {
	.dev_name = "eth0",
	.name = "kgdboe",
	.local_port = 6443,
	.remote_port = 6442,
	.remote_mac = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff },
};

static atomic_t trapped;
extern void netpoll_poll_dev(struct net_device *dev);
extern struct sk_buff *find_skb(struct netpoll *np, int len, int reserve);

static void rx_hook(struct netpoll *np, int port, char *msg, int len)
{
	int i;

	np->remote_port = port;

	/*
	 * This could be GDB trying to attach.  But it could also be GDB
	 * finishing up a session, with kgdb_connected=0 but GDB sending
	 * an ACK for the final packet.  To make sure we don't try and
	 * make a breakpoint when GDB is leaving, make sure that if
	 * !kgdb_connected the only len == 1 packet we allow is ^C.
	 */
	if (!kgdb_connected && (len != 1 || msg[0] == 3))
		kgdb_schedule_breakpoint();

	for (i = 0; i < len; i++) {
		if (msg[i] == 3)
			kgdb_schedule_breakpoint();

		if (atomic_read(&in_count) >= IN_BUF_SIZE) {
			/* buffer overflow, clear it */
			in_head = 0;
			in_tail = 0;
			atomic_set(&in_count, 0);
			break;
		}
		in_buf[in_head++] = msg[i];
		in_head &= (IN_BUF_SIZE - 1);
		atomic_inc(&in_count);
	}
}

static __sum16 checksum_udp(struct sk_buff *skb, struct udphdr *uh,
			    unsigned short ulen, __be32 saddr, __be32 daddr)
{
	__wsum psum;

	if (uh->check == 0 || skb_csum_unnecessary(skb))
		return 0;

	psum = csum_tcpudp_nofold(saddr, daddr, ulen, IPPROTO_UDP, 0);

	if (skb->ip_summed == CHECKSUM_COMPLETE &&
	    !csum_fold(csum_add(psum, skb->csum)))
		return 0;

	skb->csum = psum;

	return __skb_checksum_complete(skb);
}

static void netpoll_neigh_reply(struct sk_buff *skb, struct netpoll_info_ex *npinfo)
{
	int size, type = ARPOP_REPLY;
	__be32 sip, tip;
	unsigned char *sha;
	struct sk_buff *send_skb;
	struct netpoll *np = &kgdb_np;
	unsigned long flags;
	int hlen, tlen;
	int hits = 0, proto;

	/* Before checking the packet, we do some early
	   inspection whether this is interesting at all */
	spin_lock_irqsave(&npinfo->rx_lock, flags);
	if (np->dev == skb->dev)
		hits++;
	spin_unlock_irqrestore(&npinfo->rx_lock, flags);

	/* No netpoll struct is using this dev */
	if (!hits)
		return;

	proto = ntohs(eth_hdr(skb)->h_proto);
	if (proto == ETH_P_ARP) {
		struct arphdr *arp;
		unsigned char *arp_ptr;
		/* No arp on this interface */
		if (skb->dev->flags & IFF_NOARP)
			return;

		if (!pskb_may_pull(skb, arp_hdr_len(skb->dev)))
			return;

		skb_reset_network_header(skb);
		skb_reset_transport_header(skb);
		arp = arp_hdr(skb);

		if ((arp->ar_hrd != htons(ARPHRD_ETHER) &&
		     arp->ar_hrd != htons(ARPHRD_IEEE802)) ||
		    arp->ar_pro != htons(ETH_P_IP) ||
		    arp->ar_op != htons(ARPOP_REQUEST))
			return;

		arp_ptr = (unsigned char *)(arp+1);
		/* save the location of the src hw addr */
		sha = arp_ptr;
		arp_ptr += skb->dev->addr_len;
		memcpy(&sip, arp_ptr, 4);
		arp_ptr += 4;
		/* If we actually cared about dst hw addr,
		   it would get copied here */
		arp_ptr += skb->dev->addr_len;
		memcpy(&tip, arp_ptr, 4);

		/* Should we ignore arp? */
		if (ipv4_is_loopback(tip) || ipv4_is_multicast(tip))
			return;

		size = arp_hdr_len(skb->dev);

		spin_lock_irqsave(&npinfo->rx_lock, flags);

		{
			if (tip != np->local_ip.ip)
				return;

			hlen = LL_RESERVED_SPACE(np->dev);
			tlen = np->dev->needed_tailroom;
			send_skb = find_skb(np, size + hlen + tlen, hlen);
			if (!send_skb)
				return;

			skb_reset_network_header(send_skb);
			arp = (struct arphdr *) skb_put(send_skb, size);
			send_skb->dev = skb->dev;
			send_skb->protocol = htons(ETH_P_ARP);

			/* Fill the device header for the ARP frame */
			if (dev_hard_header(send_skb, skb->dev, ETH_P_ARP,
					    sha, np->dev->dev_addr,
					    send_skb->len) < 0) {
				kfree_skb(send_skb);
				return;
			}

			/*
			 * Fill out the arp protocol part.
			 *
			 * we only support ethernet device type,
			 * which (according to RFC 1390) should
			 * always equal 1 (Ethernet).
			 */

			arp->ar_hrd = htons(np->dev->type);
			arp->ar_pro = htons(ETH_P_IP);
			arp->ar_hln = np->dev->addr_len;
			arp->ar_pln = 4;
			arp->ar_op = htons(type);

			arp_ptr = (unsigned char *)(arp + 1);
			memcpy(arp_ptr, np->dev->dev_addr, np->dev->addr_len);
			arp_ptr += np->dev->addr_len;
			memcpy(arp_ptr, &tip, 4);
			arp_ptr += 4;
			memcpy(arp_ptr, sha, np->dev->addr_len);
			arp_ptr += np->dev->addr_len;
			memcpy(arp_ptr, &sip, 4);

			netpoll_send_skb(np, send_skb);
		}
		spin_unlock_irqrestore(&npinfo->rx_lock, flags);
	} else if( proto == ETH_P_IPV6) {
#if IS_ENABLED(CONFIG_IPV6)
		struct nd_msg *msg;
		u8 *lladdr = NULL;
		struct ipv6hdr *hdr;
		struct icmp6hdr *icmp6h;
		const struct in6_addr *saddr;
		const struct in6_addr *daddr;
		struct inet6_dev *in6_dev = NULL;
		struct in6_addr *target;

		in6_dev = in6_dev_get(skb->dev);
		if (!in6_dev || !in6_dev->cnf.accept_ra)
			return;

		if (!pskb_may_pull(skb, skb->len))
			return;

		msg = (struct nd_msg *)skb_transport_header(skb);

		__skb_push(skb, skb->data - skb_transport_header(skb));

		if (ipv6_hdr(skb)->hop_limit != 255)
			return;
		if (msg->icmph.icmp6_code != 0)
			return;
		if (msg->icmph.icmp6_type != NDISC_NEIGHBOUR_SOLICITATION)
			return;

		saddr = &ipv6_hdr(skb)->saddr;
		daddr = &ipv6_hdr(skb)->daddr;

		size = sizeof(struct icmp6hdr) + sizeof(struct in6_addr);

		spin_lock_irqsave(&npinfo->rx_lock, flags);

		{
			if (!ipv6_addr_equal(daddr, &np->local_ip.in6))
				return;

			hlen = LL_RESERVED_SPACE(np->dev);
			tlen = np->dev->needed_tailroom;
			send_skb = find_skb(np, size + hlen + tlen, hlen);
			if (!send_skb)
				return;

			send_skb->protocol = htons(ETH_P_IPV6);
			send_skb->dev = skb->dev;

			skb_reset_network_header(send_skb);
			skb_put(send_skb, sizeof(struct ipv6hdr));
			hdr = ipv6_hdr(send_skb);

			*(__be32*)hdr = htonl(0x60000000);

			hdr->payload_len = htons(size);
			hdr->nexthdr = IPPROTO_ICMPV6;
			hdr->hop_limit = 255;
			hdr->saddr = *saddr;
			hdr->daddr = *daddr;

			send_skb->transport_header = send_skb->tail;
			skb_put(send_skb, size);

			icmp6h = (struct icmp6hdr *)skb_transport_header(skb);
			icmp6h->icmp6_type = NDISC_NEIGHBOUR_ADVERTISEMENT;
			icmp6h->icmp6_router = 0;
			icmp6h->icmp6_solicited = 1;
			target = (struct in6_addr *)(skb_transport_header(send_skb) + sizeof(struct icmp6hdr));
			*target = msg->target;
			icmp6h->icmp6_cksum = csum_ipv6_magic(saddr, daddr, size,
							      IPPROTO_ICMPV6,
							      csum_partial(icmp6h,
									   size, 0));

			if (dev_hard_header(send_skb, skb->dev, ETH_P_IPV6,
					    lladdr, np->dev->dev_addr,
					    send_skb->len) < 0) {
				kfree_skb(send_skb);
				return;
			}

			netpoll_send_skb(np, send_skb);
		}
		spin_unlock_irqrestore(&npinfo->rx_lock, flags);
#endif
	}
}

static void service_neigh_queue(struct netpoll_info_ex *npi)
{
	if (npi) {
		struct sk_buff *skb;

		while ((skb = skb_dequeue(&npi->neigh_tx)))
			netpoll_neigh_reply(skb, npi);
	}
}

static bool pkt_is_ns(struct sk_buff *skb)
{
	struct nd_msg *msg;
	struct ipv6hdr *hdr;

	if (skb->protocol != htons(ETH_P_ARP))
		return false;
	if (!pskb_may_pull(skb, sizeof(struct ipv6hdr) + sizeof(struct nd_msg)))
		return false;

	msg = (struct nd_msg *)skb_transport_header(skb);
	__skb_push(skb, skb->data - skb_transport_header(skb));
	hdr = ipv6_hdr(skb);

	if (hdr->nexthdr != IPPROTO_ICMPV6)
		return false;
	if (hdr->hop_limit != 255)
		return false;
	if (msg->icmph.icmp6_code != 0)
		return false;
	if (msg->icmph.icmp6_type != NDISC_NEIGHBOUR_SOLICITATION)
		return false;

	return true;
}

int __netpoll_rx(struct sk_buff *skb, struct netpoll_info_ex *npinfo)
{
	int proto, len, ulen;
	int hits = 0;
	const struct iphdr *iph;
	struct udphdr *uh;
	struct netpoll *np = &kgdb_np;

	if (skb->dev->type != ARPHRD_ETHER)
		goto out;

	/* check if netpoll clients need ARP */
	if (skb->protocol == htons(ETH_P_ARP) && atomic_read(&trapped)) {
		skb_queue_tail(&npinfo->neigh_tx, skb);
		return 1;
	} else if (pkt_is_ns(skb) && atomic_read(&trapped)) {
		skb_queue_tail(&npinfo->neigh_tx, skb);
		return 1;
	}

	if (skb->protocol == cpu_to_be16(ETH_P_8021Q)) {
		skb = skb_vlan_untag(skb);
		if (unlikely(!skb))
			goto out;
	}

	proto = ntohs(eth_hdr(skb)->h_proto);
	if (proto != ETH_P_IP && proto != ETH_P_IPV6)
		goto out;
	if (skb->pkt_type == PACKET_OTHERHOST)
		goto out;
	if (skb_shared(skb))
		goto out;

	if (proto == ETH_P_IP) {
		if (!pskb_may_pull(skb, sizeof(struct iphdr)))
			goto out;
		iph = (struct iphdr *)skb->data;
		if (iph->ihl < 5 || iph->version != 4)
			goto out;
		if (!pskb_may_pull(skb, iph->ihl*4))
			goto out;
		iph = (struct iphdr *)skb->data;
		if (ip_fast_csum((u8 *)iph, iph->ihl) != 0)
			goto out;

		len = ntohs(iph->tot_len);
		if (skb->len < len || len < iph->ihl*4)
			goto out;

		/*
		 * Our transport medium may have padded the buffer out.
		 * Now We trim to the true length of the frame.
		 */
		if (pskb_trim_rcsum(skb, len))
			goto out;

		iph = (struct iphdr *)skb->data;
		if (iph->protocol != IPPROTO_UDP)
			goto out;

		len -= iph->ihl*4;
		uh = (struct udphdr *)(((char *)iph) + iph->ihl*4);
		ulen = ntohs(uh->len);

		if (ulen != len)
			goto out;
		if (checksum_udp(skb, uh, ulen, iph->saddr, iph->daddr))
			goto out;

		if (!(np->local_ip.ip && np->local_ip.ip != iph->daddr) &&
			!(np->remote_ip.ip && np->remote_ip.ip != iph->saddr) &&
			!(np->local_port && np->local_port != ntohs(uh->dest))) {

			rx_hook(np, ntohs(uh->source),
				       (char *)(uh+1),
				       ulen - sizeof(struct udphdr));
			hits++;
		}
	} else {
#if IS_ENABLED(CONFIG_IPV6)
		const struct ipv6hdr *ip6h;

		if (!pskb_may_pull(skb, sizeof(struct ipv6hdr)))
			goto out;
		ip6h = (struct ipv6hdr *)skb->data;
		if (ip6h->version != 6)
			goto out;
		len = ntohs(ip6h->payload_len);
		if (!len)
			goto out;
		if (len + sizeof(struct ipv6hdr) > skb->len)
			goto out;
		if (pskb_trim_rcsum(skb, len + sizeof(struct ipv6hdr)))
			goto out;
		ip6h = ipv6_hdr(skb);
		if (!pskb_may_pull(skb, sizeof(struct udphdr)))
			goto out;
		uh = udp_hdr(skb);
		ulen = ntohs(uh->len);
		if (ulen != skb->len)
			goto out;
		if (udp6_csum_init(skb, uh, IPPROTO_UDP))
			goto out;

		if (ipv6_addr_equal(&np->local_ip.in6, &ip6h->daddr) &&
			ipv6_addr_equal(&np->remote_ip.in6, &ip6h->saddr) &&
			!(np->local_port && np->local_port != ntohs(uh->dest))) {

			rx_hook(np, ntohs(uh->source),
				       (char *)(uh+1),
				       ulen - sizeof(struct udphdr));
			hits++;
		}
#endif
	}

	if (!hits)
		goto out;

	kfree_skb(skb);
	return 1;

out:
	if (atomic_read(&trapped)) {
		kfree_skb(skb);
		return 1;
	}

	return 0;
}

static rx_handler_result_t rx_handler(struct sk_buff **pskb)
{
	int ret;

	ret = __netpoll_rx(*pskb, &npinfo_ex);

	service_neigh_queue(&npinfo_ex);

	if (ret)
		return RX_HANDLER_CONSUMED;
	else
		return RX_HANDLER_PASS;
}

static void eth_pre_exception_handler(void)
{
	/* Increment the module count when the debugger is active */
	if (!kgdb_connected)
		try_module_get(THIS_MODULE);
	atomic_inc(&trapped);
}

static void eth_post_exception_handler(void)
{
	/* decrement the module count when the debugger detaches */
	if (!kgdb_connected)
		module_put(THIS_MODULE);
	atomic_dec(&trapped);
}

static int eth_get_char(void)
{
	int chr;

	while (atomic_read(&in_count) == 0) {
		netpoll_poll_dev(kgdb_np.dev);
	}

	chr = in_buf[in_tail++];
	in_tail &= (IN_BUF_SIZE - 1);
	atomic_dec(&in_count);
	return chr;
}

static void eth_flush_buf(void)
{
	if (out_count && kgdb_np.dev) {
		netpoll_send_udp(&kgdb_np, out_buf, out_count);
		memset(out_buf, 0, sizeof(out_buf));
		out_count = 0;
	}
}

static void eth_put_char(u8 chr)
{
	out_buf[out_count++] = chr;
	if (out_count == OUT_BUF_SIZE)
		eth_flush_buf();
}

static int option_setup(char *opt)
{
	char opt_scratch[MAX_CONFIG_LEN];

	configured = 0;
	/* If we're being given a new configuration, copy it in. */
	if (opt != config)
		strcpy(config, opt);

	if (opt[0] == '\0')
		return 1;

	/* But work on a copy as netpoll_parse_options will eat it. */
	strcpy(opt_scratch, opt);
	configured = !netpoll_parse_options(&kgdb_np, opt_scratch);

	return 0;
}
__setup("kgdboe=", option_setup);

/* With our config string set by some means, configure kgdboe. */
static int configure_kgdboe(void)
{
	if (option_setup(config))
		return 0;

	if (!configured) {
		printk(KERN_ERR "kgdboe: configuration incorrect - kgdboe not "
		       "loaded.\n");
		printk(KERN_ERR "  Usage: kgdboe=[src-port]@[src-ip]/[dev],"
				"[tgt-port]@<tgt-ip>/<tgt-macaddr>\n");
		return -EINVAL;
	}

	/* Bring it up. */
	if (netpoll_setup(&kgdb_np)) {
		printk(KERN_ERR "kgdboe: netpoll_setup failed kgdboe failed\n");
		return -EINVAL;
	}

	npinfo_ex.rx_flags = 0;

	spin_lock_init(&npinfo_ex.rx_lock);
	skb_queue_head_init(&npinfo_ex.neigh_tx);

	if (rtnl_trylock() &&
		netdev_rx_handler_register(kgdb_np.dev, rx_handler, NULL)) {
		printk(KERN_ERR "kgdboe: fail to register netdev rx handler\n");
		return -EINVAL;
	}

	rtnl_unlock();

	if (kgdb_register_io_module(&local_kgdb_io_ops)) {
		netpoll_cleanup(&kgdb_np);
		return -EINVAL;
	}

	configured = 2;

	return 0;
}

static int init_kgdboe(void)
{
	int ret;

	/* Already done? */
	if (configured == 2)
		return 0;

	/* OK, go ahead and do it. */
	ret = configure_kgdboe();

	if (configured == 2)
		printk(KERN_INFO "kgdboe: debugging over ethernet enabled\n");

	return ret;
}

static void cleanup_kgdboe(void)
{
	if (configured == 2)
		return;
	netpoll_cleanup(&kgdb_np);
	kgdb_unregister_io_module(&local_kgdb_io_ops);
	netdev_rx_handler_unregister(kgdb_np.dev);
	configured = 0;
}

static int param_set_kgdboe_var(const char *kmessage, struct kernel_param *kp)
{
	char kmessage_save[MAX_CONFIG_LEN];
	int len = strlen(kmessage);

	if (len >= MAX_CONFIG_LEN) {
		printk(KERN_ERR "kgdboc: config string too long\n");
		return -ENOSPC;
	}

	if (kgdb_connected) {
		printk(KERN_ERR "kgdboe: Cannot reconfigure while KGDB is "
				"connected.\n");
		return 0;
	}

	/* Start the reconfiguration process by saving the old string */
	strncpy(kmessage_save, config, sizeof(kmessage_save));


	/* Copy in the new param and strip out invalid characters so we
	 * can optionally specify the MAC.
	 */
	strcpy(config, kmessage);
	/* Chop out \n char as a result of echo */
	if (config[len - 1] == '\n')
		config[len - 1] = '\0';

	len--;
	while (len > 0 && (config[len] < ',' || config[len] > 'f')) {
		config[len] = '\0';
		len--;
	}

	if (configured == 2) {
		cleanup_kgdboe();
		configured = 0;
	}
	if (config[0] == '\0')
		return 0;

	configure_kgdboe();

	if (configured != 2)
		return -EINVAL;

	return 0;
}

static struct kgdb_io local_kgdb_io_ops = {
	.name = "kgdboe",
	.read_char = eth_get_char,
	.write_char = eth_put_char,
	.flush = eth_flush_buf,
	.pre_exception = eth_pre_exception_handler,
	.post_exception = eth_post_exception_handler
};

module_init(init_kgdboe);
module_exit(cleanup_kgdboe);
module_param_call(kgdboe, param_set_kgdboe_var, param_get_string, &kps, 0644);
MODULE_PARM_DESC(kgdboe, "[src-port]@[src-ip]/[dev],"
		 "[tgt-port]@<tgt-ip>/<tgt-macaddr>");

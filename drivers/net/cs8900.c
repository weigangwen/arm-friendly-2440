
/*
 * linux/drivers/net/cs8900.c
 *
 * Author: Abraham van der Merwe <abraham at 2d3d.co.za>
 *
 * A Cirrus Logic CS8900A driver for Linux
 * based on the cs89x0 driver written by Russell Nelson,
 * Donald Becker, and others.
 *
 * This source code is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * History:
 *    22-May-2002  Initial version (Abraham vd Merwe)
 *    30-May-2002  Added char device support for eeprom (Frank Becker)
 *    24-Jan-2004  Fixups for 2.6 (Frank Becker)
 *    15-July-2004 Modified for SMDK2410 (Roc Wu pwu at jadechip.com)
 *    7-Apr-2009 Heavily Cleaned up and modified for QQ2440 (Qinghuang Feng)
      Successful verified by FriendlyARM co. ltd. Thank you, Mr. Feng!
 */
 
#define VERSION_STRING "Cirrus Logic CS8900A driver for Linux (Modified for QQ2440)"

#include <linux/autoconf.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/platform_device.h>
#include <mach/regs-mem.h>
#include <mach/regs-gpio.h>
#include <mach/regs-irq.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/fs.h>

#include "cs8900.h"


//#define DEBUG
//#define FULL_DUPLEX
//#define DUMP_DEBUG

#ifdef DEBUG 
#define CS8900_DEBUG() printk(KERN_INFO "%s:%d\n", __func__, __LINE__) 
#else
#define CS8900_DEBUG() 
#endif

struct cs8900_priv {
	struct net_device_stats stats;
	u16 txlen;
	int char_devnum;

	void __iomem	*io_addr;	/* Register I/O base address */
	struct resource	*addr_res;   /* resources found */
	struct resource	*addr_req;   /* resources requested */
	struct resource *irq_res;
	
	spinlock_t lock;
};

/* I/O routines*/
static inline u16 cs8900_read (struct net_device *ndev,u16 reg)
{
	outw (reg,ndev->base_addr + PP_Address);
	return (inw (ndev->base_addr + PP_Data));
}

static inline void cs8900_write (struct net_device *ndev,u16 reg,u16 value)
{
	outw (reg,ndev->base_addr + PP_Address);
	outw (value,ndev->base_addr + PP_Data);
}

static inline void cs8900_set (struct net_device *ndev,u16 reg,u16 value)
{
	cs8900_write (ndev,reg,cs8900_read (ndev,reg) | value);
}

static inline void cs8900_clear (struct net_device *ndev,u16 reg,u16 value)
{
	cs8900_write (ndev,reg,cs8900_read (ndev,reg) & ~value);
}

static inline void cs8900_frame_read (struct net_device *ndev,struct sk_buff *skb,u16 length)
{
	insw (ndev->base_addr,skb_put (skb,length),(length + 1) / 2);
}

static inline void cs8900_frame_write (struct net_device *ndev,struct sk_buff *skb)
{
	outsw (ndev->base_addr,skb->data,(skb->len + 1) / 2);
}

/* Debugging functions */
#ifdef DUMP_DEBUG
static inline int printable (int c)
{
	return ((c >= 32 && c <= 126) ||
			(c >= 174 && c <= 223) ||
			(c >= 242 && c <= 243) ||
			(c >= 252 && c <= 253));
}

static void dump16 (struct net_device *ndev,const u8 *s,size_t len)
{
	int i;
	char str[128];

	if (!len) return;

	*str = '\0';

	for (i = 0; i < len; i++) {
		if (i && !(i % 4)) strcat (str," ");
		sprintf (str,"%s%.2x ",str,s[i]);
	}

	for ( ; i < 16; i++) {
		if (i && !(i % 4)) strcat (str," ");
		strcat (str,"   ");
	}

	strcat (str," ");
	for (i = 0; i < len; i++) sprintf (str,"%s%c",str,printable (s[i]) ? s[i] : '.');

	printk (KERN_DEBUG "%s:     %s\n",ndev->name,str);
}

static void hexdump (struct net_device *ndev,const void *ptr,size_t size)
{
	const u8 *s = (u8 *) ptr;
	int i;
	for (i = 0; i < size / 16; i++, s += 16) dump16 (ndev,s,16);
	dump16 (ndev,s,size % 16);
}

static void dump_packet (struct net_device *ndev,struct sk_buff *skb,const char *type)
{
	printk (KERN_INFO "%s: %s %d byte frame %.2x:%.2x:%.2x:%.2x:%.2x:%.2x to %.2x:%.2x:%.2x:%.2x:%.2x:%.2x type %.4x\n",
			ndev->name,
			type,
			skb->len,
			skb->data[0],skb->data[1],skb->data[2],skb->data[3],skb->data[4],skb->data[5],
			skb->data[6],skb->data[7],skb->data[8],skb->data[9],skb->data[10],skb->data[11],
			(skb->data[12] << 8) | skb->data[13]);
	
	if (skb->len < 0x100) 
		hexdump (ndev,skb->data,skb->len);
}
#endif	/* #ifdef  DUMP_DEBUG*/

/* Driver functions*/
static void
cs8900_release_priv(struct platform_device *pdev, struct cs8900_priv *priv)
{
	/* unmap our resources */
	iounmap(priv->io_addr);

	/* release the resources */
	release_resource(priv->addr_req);
	kfree(priv->addr_req);
}

static void cs8900_receive (struct net_device *ndev)
{
	struct cs8900_priv *priv = netdev_priv(ndev);
	struct sk_buff *skb;
	u16 status,length;
	CS8900_DEBUG(); 

	status = cs8900_read (ndev,PP_RxStatus);
	length = cs8900_read (ndev,PP_RxLength);

	if (!(status & RxOK)) {
		priv->stats.rx_errors++;
		if ((status & (Runt | Extradata))) priv->stats.rx_length_errors++;
		if ((status & CRCerror)) priv->stats.rx_crc_errors++;
		return;
	}

	if ((skb = dev_alloc_skb (length + 4)) == NULL) {
		priv->stats.rx_dropped++;
		return;
	}

	skb->dev = ndev;
	skb_reserve (skb,2);

	cs8900_frame_read (ndev,skb,length);

#ifdef FULL_DUPLEX
	dump_packet (ndev,skb,"recv");
#endif

	skb->protocol = eth_type_trans (skb,ndev);

	netif_rx (skb);
	ndev->last_rx = jiffies;

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += length;
}

static int cs8900_start_xmit (struct sk_buff *skb,struct net_device *ndev)
{
	struct cs8900_priv *priv = netdev_priv(ndev);
	u16 status;
	CS8900_DEBUG(); 

	spin_lock_irq(&priv->lock);
	netif_stop_queue (ndev);

	cs8900_write (ndev,PP_TxCMD,TxStart (After5));
	cs8900_write (ndev,PP_TxLength,skb->len);

	status = cs8900_read (ndev,PP_BusST);

	if ((status & TxBidErr)) {
		spin_unlock_irq(&priv->lock);
		printk (KERN_WARNING "%s: Invalid frame size %d!\n",ndev->name,skb->len);
		priv->stats.tx_errors++;
		priv->stats.tx_aborted_errors++;
		priv->txlen = 0;
		return 1;
	}

	if (!(status & Rdy4TxNOW)) {
		spin_unlock_irq(&priv->lock);
		printk (KERN_WARNING "%s: Transmit buffer not free!\n",ndev->name);
		priv->stats.tx_errors++;
		priv->txlen = 0;
		/* FIXME: store skb and send it in interrupt handler */
		return 1;
	}

	cs8900_frame_write (ndev,skb);
	spin_unlock_irq(&priv->lock);

#ifdef DUMP_DEBUG
	dump_packet (ndev,skb,"send");
#endif

	ndev->trans_start = jiffies;

	dev_kfree_skb (skb);

	priv->txlen = skb->len;

	return 0;
}

static irqreturn_t cs8900_interrupt (int irq,void *id)
{
	struct net_device *ndev = (struct net_device *) id;
	struct cs8900_priv *priv;
	volatile u16 status;
	irqreturn_t handled = 0;

	CS8900_DEBUG(); 
		
	priv = netdev_priv(ndev);

	if (priv == NULL) {
		printk (KERN_WARNING "%s: irq %d for unknown device.\n",ndev->name,irq);
		return 0;
	}
	
	while ((status = cs8900_read (ndev, PP_ISQ))) {
		handled = 1;

		switch (RegNum (status)) {
		case RxEvent:
			cs8900_receive (ndev);
			break;

		case TxEvent:
			priv->stats.collisions += ColCount (cs8900_read (ndev,PP_TxCOL));
			if (!(RegContent (status) & TxOK)) {
				priv->stats.tx_errors++;
				if ((RegContent (status) & Out_of_window)) priv->stats.tx_window_errors++;
				if ((RegContent (status) & Jabber)) priv->stats.tx_aborted_errors++;
				break;
			} else if (priv->txlen) {
				priv->stats.tx_packets++;
				priv->stats.tx_bytes += priv->txlen;
			}
			priv->txlen = 0;
			netif_wake_queue (ndev);
			break;

		case BufEvent:
			if ((RegContent (status) & RxMiss)) {
				u16 missed = MissCount (cs8900_read (ndev,PP_RxMISS));
				priv->stats.rx_errors += missed;
				priv->stats.rx_missed_errors += missed;
			}
			if ((RegContent (status) & TxUnderrun)) {
				priv->stats.tx_errors++;
				priv->stats.tx_fifo_errors++;

				priv->txlen = 0;
				netif_wake_queue (ndev);
			}
			/* FIXME: if Rdy4Tx, transmit last sent packet (if any) */
			break;

		case TxCOL:
			priv->stats.collisions += ColCount (cs8900_read (ndev,PP_TxCOL));
			break;

		case RxMISS:
			status = MissCount (cs8900_read (ndev,PP_RxMISS));
			priv->stats.rx_errors += status;
			priv->stats.rx_missed_errors += status;
			break;
		}
	}
	return IRQ_RETVAL(handled);
}

static void cs8900_tx_timeout (struct net_device *ndev)
{
	struct cs8900_priv *priv = netdev_priv(ndev);
	CS8900_DEBUG(); 
	priv->stats.tx_errors++;
	priv->stats.tx_heartbeat_errors++;
	priv->txlen = 0;
	netif_wake_queue (ndev);
}

static int cs8900_open(struct net_device *ndev)
{
	int result;

	set_irq_type(ndev->irq, IRQ_TYPE_EDGE_RISING);
	/* enable the ethernet controller */
	cs8900_set (ndev,PP_RxCFG,RxOKiE | BufferCRC | CRCerroriE | RuntiE | ExtradataiE);
	cs8900_set (ndev,PP_RxCTL,RxOKA | IndividualA | BroadcastA);
	cs8900_set (ndev,PP_TxCFG,TxOKiE | Out_of_windowiE | JabberiE);
	cs8900_set (ndev,PP_BufCFG,Rdy4TxiE | RxMissiE | TxUnderruniE | TxColOvfiE | MissOvfloiE);
	cs8900_set (ndev,PP_LineCTL,SerRxON | SerTxON);
	cs8900_set (ndev,PP_BusCTL,EnableRQ);

#ifdef FULL_DUPLEX
	cs8900_set (ndev,PP_TestCTL,FDX);
#endif

	udelay(200);	

	if ((result = request_irq (ndev->irq, cs8900_interrupt, 0, ndev->name, ndev)) < 0) {
		printk (KERN_INFO "%s: could not register interrupt %d\n",ndev->name, ndev->irq);
		return (result);
	}

	netif_start_queue (ndev);

	return 0;
}

static int cs8900_close(struct net_device *ndev)
{
	printk(KERN_INFO "cs8900: close...\n");
	/* disable ethernet controller */
	cs8900_write (ndev,PP_BusCTL,0);
	cs8900_write (ndev,PP_TestCTL,0);
	cs8900_write (ndev,PP_SelfCTL,0);
	cs8900_write (ndev,PP_LineCTL,0);
	cs8900_write (ndev,PP_BufCFG,0);
	cs8900_write (ndev,PP_TxCFG,0);
	cs8900_write (ndev,PP_RxCTL,0);
	cs8900_write (ndev,PP_RxCFG,0);

	free_irq (ndev->irq,ndev);

	netif_stop_queue (ndev);

	return 0;
}

static struct net_device_stats *cs8900_get_stats(struct net_device *ndev)
{
	struct cs8900_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &priv->stats;
	
	return stats;
}

static void cs8900_set_receive_mode (struct net_device *ndev)
{
	if ((ndev->flags & IFF_PROMISC))
		cs8900_set (ndev,PP_RxCTL,PromiscuousA);
	else
		cs8900_clear (ndev,PP_RxCTL,PromiscuousA);

	if ((ndev->flags & IFF_ALLMULTI) && ndev->mc_list)
		cs8900_set (ndev,PP_RxCTL,MulticastA);
	else
		cs8900_clear (ndev,PP_RxCTL,MulticastA);
}


static const struct net_device_ops cs8900_netdev_ops = {
	.ndo_open	= cs8900_open,
	.ndo_stop		= cs8900_close,
	.ndo_start_xmit	= cs8900_start_xmit,
	.ndo_get_stats		= cs8900_get_stats,
	.ndo_set_multicast_list 	= cs8900_set_receive_mode,
	.ndo_tx_timeout	= cs8900_tx_timeout,
};

/*Driver initialization routines*/
static int __devinit 
cs8900_probe (struct platform_device *pdev)
{
	int i;
	u16 value;
	int ret;
	int iosize;
	struct net_device *ndev;
	struct cs8900_priv *priv;

 	ndev = alloc_etherdev(sizeof(struct cs8900_priv));
	if (!ndev) {
		dev_err(&pdev->dev, "cs8900: could not allocate device.\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);

	printk(KERN_INFO "cs8900: probbing for qq2440...\n");
	
	priv = netdev_priv(ndev);
	memset(priv, 0, sizeof(*priv));
		
	__raw_writel(0x2211d110,S3C2410_BWSCON);//!!
	__raw_writel(0x1f7c,S3C2410_BANKCON3);//!!
		
	ndev->dev_addr[0] = 0x08;
	ndev->dev_addr[1] = 0x00;
	ndev->dev_addr[2] = 0x3e;
	ndev->dev_addr[3] = 0x26;
	ndev->dev_addr[4] = 0x0a;
	ndev->dev_addr[5] = 0x5b;

	ndev->if_port   = IF_PORT_10BASET;
	
	spin_lock_init(&priv->lock);

	priv->addr_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->irq_res= platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (priv->addr_res == NULL ||priv->irq_res == NULL) {
		printk(KERN_ERR "cs8900: Can't get cs8900 device resource.\n");
		ret = -ENOENT;
		goto out;
	}

	iosize = priv->addr_res->end -  priv->addr_res->start + 1;
	
	priv->addr_req = request_mem_region(priv->addr_res->start, iosize,
					  				pdev->name);
	
	if (priv->addr_req == NULL) {
		printk(KERN_ERR  "cs8900: cannot claim address reg area.\n");
		ret = -EIO;
		goto out;
	}

	priv->io_addr = ioremap(priv->addr_res->start, iosize);

	if (priv->io_addr == NULL) {
		printk(KERN_ERR  "cs8900: failed to ioremap address reg\n");
		ret = -EINVAL;
		goto out;
	}

	/* fill in parameters for net-dev structure */
	ndev->base_addr = (unsigned long)priv->io_addr;
	ndev->irq	= priv->irq_res->start;
	
	/* verify EISA registration number for Cirrus Logic */
	if ((value = cs8900_read (ndev,PP_ProductID)) != EISA_REG_CODE) {
		printk (KERN_ERR "%s: incorrect signature 0x%.4x\n",ndev->name,value);
		return -ENXIO;
	}

	/* verify chip version */
	value = cs8900_read (ndev,PP_ProductID + 2);
	if (VERSION (value) != CS8900A) {
		printk (KERN_ERR "%s: unknown chip version 0x%.8x\n",ndev->name,VERSION (value));
		return -ENXIO;
	}
	/* setup interrupt number */
	cs8900_write (ndev,PP_IntNum,0);

	printk (KERN_INFO "%s: CS8900A rev %c at %#lx irq=%d",
		ndev->name,'B' + REVISION (value) - REV_B, ndev->base_addr, ndev->irq);

	/* qq2440 doesn't have eeprom, configure MAC address by hand */
	for (i = 0; i < ETH_ALEN; i += 2)
		cs8900_write (ndev,PP_IA + i,ndev->dev_addr[i] | (ndev->dev_addr[i + 1] << 8));

	printk ( ", addr:");
	for (i = 0; i < ETH_ALEN; i += 2)
	{
		u16 mac = cs8900_read (ndev,PP_IA + i);
		printk( "%c%02X:%2X", (i==0)?' ':':', mac & 0xff, (mac >> 8));
	}
	printk("\n");

	printk(KERN_INFO "[cs89x0_probe] S3C2410_BWSCON = %x\r\n", __raw_readl(S3C2410_BWSCON));
	printk(KERN_INFO "[cs89x0_probe] S3C2410_BANKCON3 = %x\r\n", __raw_readl(S3C2410_BANKCON3));
	printk(KERN_INFO "[cs89x0_probe] S3C2410_GPGCON = %x\r\n", __raw_readl(S3C2410_GPGCON));
	printk(KERN_INFO "[cs89x0_probe] S3C2410_EXTINT1 = %x\r\n", __raw_readl(S3C2410_EXTINT1));
	printk(KERN_INFO "[cs89x0_probe] S3C2410_EINTMASK = %x\r\n", __raw_readl(S3C2410_EINTMASK));

	ndev->netdev_ops = &cs8900_netdev_ops;
	ndev->watchdog_timeo     = HZ;

	ret = register_netdev(ndev);
	if (!ret)
		return 0;

out:
	printk(KERN_ERR  "cs8900: probe failed!\n");

	cs8900_release_priv(pdev, priv);
	free_netdev(ndev);

	return ret;
}

static int __devexit 
cs8900_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct cs8900_priv *priv = netdev_priv(ndev);

	platform_set_drvdata(pdev, NULL);

	unregister_netdev(ndev);
	cs8900_release_priv(pdev, priv);
	free_netdev(ndev);
	
	printk(KERN_INFO "cs8900: released and freed device\n");
	return 0;
}

static struct platform_driver cs8900_driver = {
	.driver	= {
		.name    = "cs8900",
		.owner	 = THIS_MODULE,
	},
	.probe   = cs8900_probe,
	.remove  = __devexit_p(cs8900_drv_remove),
	//.suspend = cs8900_drv_suspend,
	//.resume  = cs8900_drv_resume,
};

static __init int cs8900_init(void)
{
	printk(KERN_INFO "Register cs8900 Ethernet Driver for qq2440.\n");
	return platform_driver_register(&cs8900_driver);
}

static __exit void cs8900_exit(void)
{
	platform_driver_unregister(&cs8900_driver);
}

module_init(cs8900_init);
module_exit(cs8900_exit);

MODULE_AUTHOR("Modified by Qinghuang Feng <qhfeng.kernel@gmail.com>");
MODULE_DESCRIPTION("CS8900 driver cleaned up and modified for qq2440");
MODULE_LICENSE("GPL");

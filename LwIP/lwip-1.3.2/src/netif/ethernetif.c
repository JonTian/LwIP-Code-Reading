/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

/*
 * This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
 */

#include "lwip/opt.h"

#if 1 /* don't build, this is only a skeleton, see previous comment */

/* lwIP includes. */
#include <string.h>
#include "stm32f10x.h"
#include "enc28j60.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"
#include "netif/ppp_oe.h"
  

/* Define those to better describe your network interface. */
#define IFNAME0 'e'
#define IFNAME1 'n'

#define netifMTU				( 1500 )
#define netifINTERFACE_TASK_STACK_SIZE		( 350 )
#define netifGUARD_BLOCK_TIME			( 250 )


extern  uint8_t macaddress[6];	   //在 LWIP_Init.c文件中定义
unsigned char Data_Buf[1520 *4 +1];
unsigned char Tx_Data_Buf[1500 *2 ];

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
/* lwIP definitions. */
struct ethernetif
{
	struct eth_addr *ethaddr;
};

/* Forward declarations. */
err_t  ethernetif_input(struct netif *netif);
//static err_t ethernetif_output( struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr );
err_t ethernetif_init( struct netif *netif );
extern void enc28j60PacketSend(u16 len, u8* packet);


/*-----------------------------------------------------------*/
     //网卡初始化函数
/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
        struct ethernetif *ethernetif = netif->state;  
	/* set MAC hardware address length */
	netif->hwaddr_len = 6;

	/* set MAC hardware address */
	/* MAC地址 */
	netif->hwaddr[0] = macaddress[0];
	netif->hwaddr[1] = macaddress[1];
	netif->hwaddr[2] = macaddress[2];
	netif->hwaddr[3] = macaddress[3];
	netif->hwaddr[4] = macaddress[4];
	netif->hwaddr[5] = macaddress[5];

	/* maximum transfer unit */
	/* 最大传输单元 */
	netif->mtu = netifMTU;
        
        /* device capabilities */
        /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

	/* Initialise the EMAC.  This routine contains code that polls status bits.  
	If the Ethernet cable is not plugged in then this can take a considerable 
	time.  To prevent this starving lower priority tasks of processing time we
	lower our priority prior to the call, then raise it back again once the
	initialisation is complete. */
        
        /* Do whatever else is needed to initialize interface. */ 
	enc28j60Init(netif->hwaddr);	  //初始化enc28j60
        //enc28j60PhyWrite(PHLCON, 0x476);  //设置PHY
        enc28j60clkout(1); 				// change clkout from 6.25MHz to 12.5MHz
}	


/*-----------------------------------------------------------*/
/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become availale since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output( struct netif *netif, struct pbuf *p )  /*底层发送数据函数*/
{   
  struct ethernetif *ethernetif = netif->state;
  struct pbuf *q;
  
  int i = 0;  
  err_t xReturn = ERR_OK;
  
#if ETH_PAD_SIZE
  pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif
  
  /* Parameter not used. */
  for(q = p; q != NULL; q = q->next) 
  {
        /* Send the data from the pbuf to the interface, one pbuf at a
       time. The size of the data in each pbuf is kept in the ->len
       variable. */
        memcpy(&Tx_Data_Buf[i], (u8_t*)q->payload, q->len); // ping -t 192.168.1.15
   	i = i + q->len;
  }
  
  enc28j60PacketSend(i,Tx_Data_Buf); //发送数据包

#if ETH_PAD_SIZE
  pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif
    
  return xReturn;
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *
low_level_input(struct netif *netif)
{	                                                   
        struct ethernetif *ethernetif = netif->state;
	struct pbuf *q,*p = NULL;
	u16 Len = 0; 

  	int i =0;
        
	/* Obtain the size of the packet and put it into the "len" variable. */		
	Len = enc28j60PacketReceive(1520 *4,  Data_Buf);	//返回接收到的数据包长度
        
#if ETH_PAD_SIZE
        len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif
	if ( Len == 0 ) return 0;
        /* We allocate a pbuf chain of pbufs from the pool. */				
	p = pbuf_alloc(PBUF_RAW, Len, PBUF_POOL);
		
	if (p != NULL) 
	{
          

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE); /* drop the padding word */
#endif

    /* We iterate over the pbuf chain until we have read the entire
     * packet into the pbuf. */
            for(q = p; q != NULL; q = q->next) {
                     memcpy((u8_t*)q->payload, (u8_t*)&Data_Buf[i], q->len);

                     i = i + q->len;
            }
            if( i != p->tot_len ){ return 0;}  //相等时表明到了数据尾
        }
        return p;
    

#if ETH_PAD_SIZE
        pbuf_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif



    
}
/*-----------------------------------------------------------*/

/*
 * ethernetif_output(): This function is called by the TCP/IP stack when an 
 * IP packet should be sent. It calls the function called low_level_output() 
 * to do the actual transmission of the packet.
 */
//static err_t ethernetif_output( struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr )
//{
//    /* resolve hardware address, then send (or queue) packet */
//    return etharp_output( netif, p, ipaddr );
//}
/*-----------------------------------------------------------*/

	
/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
err_t 
ethernetif_input(struct netif *netif) 
{  	
 err_t err = ERR_OK;		
 struct pbuf *p = NULL;
						  			
   /* move received packet into a new pbuf */
  p = low_level_input(netif);

  if (p == NULL) return ERR_MEM;

  err = netif->input(p, netif);
  if (err != ERR_OK)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
    pbuf_free(p);
    p = NULL;
  }

  return err;
}
  
/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init( struct netif *netif )
{
struct ethernetif   *ethernetif;

	ethernetif = mem_malloc( sizeof(struct ethernetif) );

	if( ethernetif == NULL )
	{
		LWIP_DEBUGF( NETIF_DEBUG, ("ethernetif_init: out of memory\n\r") );
		return ERR_MEM;
	}

	netif->state = ethernetif;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;

	ethernetif->ethaddr = ( struct eth_addr * ) &( netif->hwaddr[0] );

	low_level_init( netif );
	
	return ERR_OK;
}

#endif /* 0 */
/*************************************************************************************************************/



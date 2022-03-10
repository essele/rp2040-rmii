/**
 * @file rmii_lwip.c
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "clock.pio.h"

#include "mdio.h"
#include "mac_tx.h"
#include "mac_rx.h"


#include "lwip/def.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/etharp.h"
#include "lwip/snmp.h"
#include "netif/ethernet.h"
#include "lwip/timeouts.h"

#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"

static uint8_t fake_mac[ 6 ] = {
    0xa4,0xdd,0x7b,0xb6,0xf2,0x1d
};

// Specific structure for our interface
struct ethernetif {
    int             xx;
};

/**
 * @brief Send a link-level packet from LWIP
 * 
 * @param netif 
 * @param p 
 * @return err_t 
 */
static err_t rmii_linkoutput(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;

#if ETH_PAD_SIZE
    pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif
/*
    // For the moment we'll just pull all the pbufs together into
    // a new buffer and send that
    uint8_t buffer[1600];

    uint len = 0;
    for(q=p; q != NULL; q=q->next) {
        memcpy(&buffer[len], q->payload, q->len);
        len += q->len;
    }
    if (len < 60) len = 60;
    mac_tx_send(buffer, len);
*/
    mac_tx_send_pbuf(p);
#if ETH_PAD_SIZE
    pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    return ERR_OK;
}

err_t ethernetif_init(struct netif *netif)
{
  struct ethernetif *ethernetif;

  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   * 
   * TODO: do we update this when the link goes up and down?
   */
  MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, 100000000);

  netif->name[0] = 'e';
  netif->name[1] = 'n';
  /* We directly use etharp_output() here to sethernetifave a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
#if LWIP_IPV4
  netif->output = etharp_output;
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
  netif->linkoutput = rmii_linkoutput;


    /* initialize the hardware */
    // TODO: init here rather than in main!

    netif->hwaddr_len = ETHARP_HWADDR_LEN;
    memcpy(netif->hwaddr, fake_mac, ETHARP_HWADDR_LEN);
 
    netif->mtu = 1500;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
  return ERR_OK;
}

// Custom pbuf structure for holding our rx frames...
typedef struct rmii_pbuf {
    struct pbuf_custom  p;
    struct rx_frame     *frame;
} rmii_pbuf_t;

// TODO: count should be our buffer count...
LWIP_MEMPOOL_DECLARE(RX_POOL, RX_FRAME_COUNT, sizeof(rmii_pbuf_t), "Zero-copy RX PBUF pool");

// Custom pbuf freeing routine...
void rmii_pbuf_free(struct pbuf *p) {
//  SYS_ARCH_DECL_PROTECT(old_level);

  rmii_pbuf_t* rmii_pbuf = (rmii_pbuf_t*)p;
  rx_add_to_free_list(rmii_pbuf->frame);
  LWIP_MEMPOOL_FREE(RX_POOL, rmii_pbuf);
//  printf("rmii pbuf returned\r\n");
}



/**
 * @brief Check for incoming packets and hand them to lwip
 * 
 * This is called regularly in the main loop to ensure incoming packets are
 * dealt with.
 * 
 * TODO: should loop until we've done all packets otherwise we might
 * be a bit slow if we get bombarded
 */
void rmii_lwip_poll(struct netif* netif) {
    struct rx_frame *frame;
    struct pbuf *p = NULL;
    struct pbuf *q;

    rmii_pbuf_t *cp;
    
    while((frame = rx_get_ready_frame())) {
        // Try to allocate some pbufs for the frame, if not give the frame back and
        // return...
        /*
        p = pbuf_alloc(PBUF_RAW, frame->length, PBUF_POOL);
        if (!p) {
            rx_add_to_free_list(frame);
            printf("PBUF ALLOCATION FAILED.\r\n");
            return;
        }
        */
        cp = (rmii_pbuf_t *)LWIP_MEMPOOL_ALLOC(RX_POOL);
        if (!cp) {
            rx_add_to_free_list(frame);
            printf("PBUF (CUSTOM) allocation failed\r\n");
            return;
        }
        cp->p.custom_free_function = rmii_pbuf_free;
        cp->frame = frame;

        p = pbuf_alloced_custom(PBUF_RAW, frame->length, PBUF_REF, &cp->p, frame->data, RX_MAX_BYTES);
        if (!p) {
            printf("PBUF ERROR\r\n");
            panic("pbuf failed\r\n");   
        }
/*


        // See if we're working...
    //    printf("MAIN (size=%d / addr=%08x / fcs=%08x):", frame->length, frame, frame->checksum);
    //    for (int i=0; i < 16; i++) {
    //        printf(" %02x", frame->data[i]);
    //    }
    //    printf("\r\n");

        // Now copy the packet into the pbufs ... need to not do this!
        int len = 0;
        for(q=p; q != NULL; q=q->next) {
            memcpy(q->payload, &frame->data[len], q->len);
            len += q->len;
        }

        // We're done with the frame at this point
        rx_add_to_free_list(frame);
*/
        // Update SNMP stats...
        MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
        if (((uint8_t *)p->payload)[0] & 1) {
            MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
        } else {
            MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
        }
        LINK_STATS_INC(link.recv);

        // Pass the frame into lwip
        if (netif->input(p, netif) != ERR_OK) {
            LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
            pbuf_free(p);
            p = NULL;
        }
    }
    // Once we've dealt with any queued frames we can have a look and
    // see if our link state has changed...
    int link_change = mdio_poll();
    if (link_change) {
        switch(link_change) {
            case LINK_DOWN:
                netif_set_link_down(netif);
                break;

            case LINK_UP_10FD:
            case LINK_UP_10HD:
            case LINK_UP_100FD:
            case LINK_UP_100HD:
                netif_set_link_up(netif);
                break;

            default:
                break;
        }
    }

    // And update the LWIP timeouts...
    sys_check_timeouts();
}

struct netif *rmii_lwip_init(uint clkmhz, uint rx0, uint rx1, uint crs, uint tx0, uint tx1, uint txen, uint mdclk, uint mdio) {
    assert(clkmhz == 100 || clkmhz == 150);
    assert(rx1 == rx0 + 1);
    assert(tx1 == tx0 + 1);

    //
    // New approach ... just clock the pin using the clk_sys value then we will hopefully
    // be in sync and not have any variation in phase between clk_sys and the output.
    //
    // NOTE: I'm assuming this will also work for 150MHz but we'll need to use the duty
    // cycle fix thing because of the divide by odd number (3).
    //
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 2);
    clocks_hw->clk[clk_gpout0].ctrl |= CLOCKS_CLK_GPOUT0_CTRL_DC50_BITS;

    // We prioritise DMA... both read and write...
    //bus_ctrl_hw->priority |= (1 << 12) | (1 << 8);

    // Initialise the TX and RX modules...
//    mac_tx_init(13, 15, 28);     // tx0=13, tx1=14, txen=15, crs=28
//    mac_rx_init(26, 28);         // rx0=26, rx1=27, crs=28
    mac_tx_init(tx0, txen, crs);
    mac_rx_init(rx0, crs);

    // And start MDIO, monitor link status and speed, and load the
    // relevant tx and rx PIO modules as needed...
//    mdio_init(18, 19);
    mdio_init(mdclk, mdio);

    // Timer 3 is running by default and it really high priority, so we need to
    // look at whether disabling this helps or not...
    irq_set_enabled(TIMER_IRQ_3, false);


    printf("rmii init complete.\r\n");

    static struct netif rmii_netif;
    static struct ethernetif rmii_ethernetif;
    struct netif* nif;

    lwip_init();

    LWIP_MEMPOOL_INIT(RX_POOL);

    rmii_netif.state = &rmii_ethernetif;
    nif = netif_add_noaddr(&rmii_netif, &rmii_ethernetif, ethernetif_init, ethernet_input);

    // Set the interface as up, but start with the link down, this will be updated
    // by the mdio polling system...
    netif_set_up(&rmii_netif);
    netif_set_link_down(&rmii_netif);

    return nif;
}



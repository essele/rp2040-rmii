#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
//#include "hardware/pio.h"
//#include "hardware/dma.h"
//#include "hardware/irq.h"
#include "clock.pio.h"


#include "mdio.h"
#include "mac_tx.h"
#include "mac_rx.h"

const uint LED_PIN = 25;

void my_clocks_init(void) {
    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    hw_clear_bits(&clocks_hw->clk[clk_sys].ctrl, CLOCKS_CLK_SYS_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_sys].selected != 0x1)
        tight_loop_contents();
    hw_clear_bits(&clocks_hw->clk[clk_ref].ctrl, CLOCKS_CLK_REF_CTRL_SRC_BITS);
    while (clocks_hw->clk[clk_ref].selected != 0x1)
        tight_loop_contents();

    /// \tag::pll_settings[]
    // Configure PLLs
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 3 / 1 = 500MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
    /// \end::pll_settings[]

    /// \tag::pll_init[]
    pll_init(pll_sys, 1, 1500 * MHZ, 3, 1);
    pll_init(pll_usb, 1, 480 * MHZ, 5, 2);
    /// \end::pll_init[]
   // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    0, // No aux mux
                    12 * MHZ,
                    12 * MHZ);

    /// \tag::configure_clk_sys[]
    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    500 * MHZ,
                    100 * MHZ);
    /// \end::configure_clk_sys[]

    // CLK USB = PLL USB (48MHz) / 1 = 48MHz
    clock_configure(clk_usb,
                    0, // No GLMUX
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    clock_configure(clk_adc,
                    0, // No GLMUX
                    CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    clock_configure(clk_rtc,
                    0, // No GLMUX
                    CLOCKS_CLK_RTC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    46875);

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    100 * MHZ,
                    100 * MHZ);
}

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

    // For the moment we'll just pull all the pbufs together into
    // a new buffer and send that
    uint8_t buffer[1600];

    uint len = 0;
    for(q=p; q != NULL; q=q->next) {
        memcpy(&buffer[len], q->payload, q->len);
        len += q->len;
    }
    while (len < 60) {
        buffer[len++] = 0;
    }
    mac_tx_send(buffer, len);
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
}


int main() {
    my_clocks_init();

    stdio_init_all();  
   
    gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

    // If we are running at 500MHz PLL output, then we need a /10 output for
    // the 50MHz PHY clock
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

    // Initialise the TX and RX modules...
    mac_tx_init(13, 15, 28);     // tx0=13, tx1=14, txen=15, crs=28
    mac_rx_init(26, 28);         // rx0=26, rx1=27, crs=28

    // And start MDIO, monitor link status and speed, and load the
    // relevant tx and rx PIO modules as needed...
    mdio_init(18, 19);  

    printf("rmii init complete.\r\n");


    static struct netif rmii_netif;
    static struct ethernetif rmii_ethernetif;
    struct netif* nif;

    lwip_init();

    LWIP_MEMPOOL_INIT(RX_POOL);

    // Setup rmii_ethernetif -- this will be passed into the other routines so
    // we can init our one specifically...
    // TODO: set pins etc.

    rmii_netif.state = &rmii_ethernetif;
    nif = netif_add_noaddr(&rmii_netif, &rmii_ethernetif, ethernetif_init, ethernet_input);

    netif_set_up(&rmii_netif);
    netif_set_link_up(&rmii_netif);
    dhcp_start(&rmii_netif);

    //httpd_init();

    uint8_t prevDHCPState;

    while( true )
    {
        sleep_us( 1 );
        sys_check_timeouts();

#ifndef MDIO_USE_IRQ
        mdio_poll();
#endif

        rmii_lwip_poll(nif);
        // show DHCP status
        struct dhcp* dd = netif_dhcp_data(&rmii_netif);
        if(dd->state != prevDHCPState) {
            printf("DHCP State goes from %d to %d\n", prevDHCPState, dd->state);
            prevDHCPState = dd->state;
            if(dd->state == DHCP_STATE_BOUND) {
                char tmp[ 256 ];
                printf(" Got address: \n" );
                printf("   IP     : %s\n", ipaddr_ntoa_r(&dd->offered_ip_addr, tmp, sizeof(tmp)));
                printf("   Subnet : %s\n", ipaddr_ntoa_r(&dd->offered_sn_mask, tmp, sizeof(tmp)));
                printf("   GW     : %s\n", ipaddr_ntoa_r(&dd->offered_gw_addr, tmp, sizeof(tmp)));
            }
        }
    }



    // Now send 100 packets...
    for (int i=0; i < 100; i++) {
        sleep_ms(100);
        //mac_tx_test();
    }
    while(1);

}

#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"


#include "rmii_lwip.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"

//#include "mdio.h"

const uint LED_PIN = 25;



int main() {
    // Set the system clock so we are a multiple of 50Mhz, this will impact the
    // PIO state machines to the code will be different for different frequencies.
    set_sys_clock_khz(100000, true);

    stdio_init_all();  
   
    gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);


    struct netif *nif = rmii_lwip_init(100, 26, 27, 28, 13, 14, 15, 18, 19);



/*
    // Let's just send 5000 packets
//    sleep_ms(3000);
    printf("Sending 100000 packets\r\n");
    for(int i=0; i < 100000; i++) {
        mac_tx_test();
    }
    printf("\r\ndone.\r\n");
    while(1);
*/
    //httpd_init();

    uint64_t last_perf = 0;
    uint8_t prevDHCPState;

    dhcp_start(nif);

    while( true )
    {
        uint64_t now = time_us_64();
        if (now > last_perf + 1000000) {
            last_perf = now;
            print_rx_stats();
        }

        //sleep_us( 1 );
//        sys_check_timeouts();



        rmii_lwip_poll(nif);
        // show DHCP status
        struct dhcp* dd = netif_dhcp_data(nif);
//        struct dhcp* dd = netif_dhcp_data(&rmii_netif);
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

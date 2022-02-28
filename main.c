#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "clock.pio.h"
#include "macrx.pio.h"

#include "mdio.h"

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

int rx_dma_chan;

uint8_t rx_frame[2048];

#define RX_DMA_LENGTH         1200

volatile int flag = 0;



void isr()
{
    //printf("In here");
    // State machine number impacts the IRQ...
    pio_interrupt_clear(pio0, 0);
    pio_interrupt_clear(pio0, 1);
    irq_clear(PIO0_IRQ_0);
    flag = 1;
}

void dma_isr() {
    dma_channel_acknowledge_irq0(rx_dma_chan);
    printf("DMA OVERRUN");
}

int main() {
    my_clocks_init();

    stdio_init_all();


    gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

    // If we are running at 500MHz PLL output, then we need a /10 output for
    // the 50MHz PHY clock
    //gpio_set_drive_strength(21, GPIO_DRIVE_STRENGTH_4MA);
    //gpio_set_slew_rate(21, GPIO_SLEW_RATE_FAST);
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 10);

    PIO pio = pio0;
    uint offset;
    uint sm;

/*    
    offset = pio_add_program(pio, &clock_program);
    sm = pio_claim_unused_sm(pio, true);

    gpio_put(19, 0);            // drive a 0 always
    mmio_program_init(pio, sm, offset, 18, 19);        // will start
*/
    mdio_init(18, 19);  

    sleep_ms(200);

    // Try to get it to autoneg at 100MBPS

    uint32_t rc;

    mmio_read(1, MMIO_BASIC_CONTROL);
    mmio_read(1, MMIO_AUTONEG);
    mmio_write(1, MMIO_BASIC_CONTROL, 0x1200);
    mmio_read(1, MMIO_BASIC_CONTROL);

    for(int i=0; i < 10; i++) {
        // Get status...
        rc = mmio_read(1, MMIO_BASIC_STATUS);
        printf("Status Value -- %08x\r\n", rc);
        sleep_ms(200);
    }


    sleep_ms(500);



    //
    // Now the RX side
    //
    offset = pio_add_program(pio, &macrx_program);
    sm = pio_claim_unused_sm(pio, true);

    // We need pull downs enabled on the rx pins, I assume this is to ensure it
    // gets back to 0 quickly enough.
    // TODO: actually this may not be a problem, I suspect this is more about
    //       the power-on defaults and having different settings. (Not sure how
    //       the device actually does a reset during debugging though.)
//    gpio_disable_pulls(26);
//    gpio_disable_pulls(27);
//    gpio_disable_pulls(28);
    gpio_pull_down(26);
    gpio_pull_down(27);
    gpio_pull_down(28);

    // this one doesn't start it...
    macrx_program_init(pio, sm, offset, 26, 28);    // rx0=26, rx1=27, csr=28


    // We have a DMA complete interrupt which will signal if we overrun a buffer
    // Normally it won't complete, the PIO interrupt is the signal the packet
    // is done, and we abort DMA at that point.
    //
    // If DMA overruns then we need to stop and restart the PIO.
    // If PIO finishes, we just need to lower the IRQ it will be waiting on.

    dma_channel_config rx_dma_channel_config;

    rx_dma_chan = dma_claim_unused_channel(true);
    rx_dma_channel_config = dma_channel_get_default_config(rx_dma_chan);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_irqn_set_channel_enabled(0, rx_dma_chan, true);



        
    channel_config_set_read_increment(&rx_dma_channel_config, false);
    channel_config_set_write_increment(&rx_dma_channel_config, true);
    channel_config_set_dreq(&rx_dma_channel_config, pio_get_dreq(pio, sm, false));
    channel_config_set_transfer_data_size(&rx_dma_channel_config, DMA_SIZE_8);

    dma_channel_configure(
        rx_dma_chan, &rx_dma_channel_config,
        rx_frame,
        ((uint8_t*)&pio->rxf[sm]) + 3,
        RX_DMA_LENGTH,
        false
    );

    irq_set_exclusive_handler(PIO0_IRQ_0, isr);
    irq_set_enabled(PIO0_IRQ_0, true);


    dma_channel_start(rx_dma_chan);
    pio_sm_set_enabled(pio, sm, true);

    dma_channel_hw_t *hw = dma_channel_hw_addr(rx_dma_chan);
    while(1) {
        uint64_t now = time_us_64();
        while(!flag) {
            if (time_us_64() > now + 3000000) {
                int pc = pio_sm_get_pc(pio, sm);
                printf("No data in 3s ... offset=%d pc=%d\r\n", offset, pc);
                printf("rx_frame=%08x rx_addr=%08x\r\n", rx_frame, hw->write_addr);
                now = time_us_64();
            }
        }
        //while(!flag);       // wait for the flag
        //flag = 0;
        printf("About to abort: ");
        dma_channel_abort(rx_dma_chan);
        for(int i=0; i < 10; i++) {
            printf(".");
            sleep_us(10);
        }
        printf("\r\n");
        pio_sm_set_enabled(pio, sm, false);

        uint32_t rxaddr = hw->write_addr;
        int len = rxaddr - ((uint32_t)rx_frame);

        printf("Have %d bytes:", len);
        for(int i=0; i < 32; i++) {
            printf(" %02x", rx_frame[i]);
            rx_frame[i] = 0;
        }
//        printf("\r\n");
        printf("  SNIFF = %08x\r\n", dma_hw->sniff_data);
        if (len > 1600) {
            printf("STOPPING AT LARGE PACKET\r\n");
            while(1);
        }
        dma_channel_configure(
            rx_dma_chan, &rx_dma_channel_config,
            rx_frame,
            ((uint8_t*)&pio->rxf[sm]) + 3,
            RX_DMA_LENGTH,
            false
        );
        dma_sniffer_enable(rx_dma_chan, 0x1, true);
        
        dma_hw->sniff_data = 0xffffffff;

        dma_channel_start(rx_dma_chan);
        macrx_program_init(pio, sm, offset, 26, 28);    // rx0=26, rx1=27, csr=28

        flag = 0;
        pio_sm_set_enabled(pio, sm, true);
    }

    while(1);

    sleep_ms(2000);

    for(int i=0; i < 32; i++) {
        printf("At location: %02x --> %02x\r\n", i, rx_frame[i]);
    }

    printf("Offser is %d\r\n", offset);
    uint8_t pc = pio_sm_get_pc(pio, sm);
    printf("Pc is %d\r\n", pc);

    dma_channel_config xx = dma_get_channel_config(rx_dma_chan);

    while(1);




    while(1) {}

    while(1) {
        gpio_put(LED_PIN, 0);
        sleep_ms(500);
        gpio_put(LED_PIN, 1);
        sleep_ms(500);
    }
   while(1) {
   }
}

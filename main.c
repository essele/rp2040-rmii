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
//#include "mac_tx.pio.h"

#include "mdio.h"
#include "mac_tx.h"

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


PIO                 rx_pio = pio0;          // Which PIO are we running on for RX
uint                rx_sm;                 // State machine for RX
uint                rx_offset;             // Start of the RX code

dma_channel_hw_t    *rx_dma_chan_hw;
int                 rx_dma_chan;
uint8_t             rx_frame[2048];

#define RX_DMA_LENGTH         1600




volatile int flag = 0;


//
// This is called when the state machine reaches the end of a packet
//
// So ... stop the sm, clear the sm irq, abort the dma, then setup
// everything new again.
//
// This is a higher priority isr, so there could be a dma IRQ pending
// which we need to clear, we should also avoid creating one by aborting
// the dma.
//
// Suggestion: before aborting the dma turn off the DMA irq.
//
//
void pio_rx_isr() {
    uint32_t length;

    // First disable the DMA irq from happening when we abort...
    dma_channel_set_irq0_enabled(rx_dma_chan, false);

    // Now abort the DMA channel... (this won't wait for outstanding xfers)
    dma_hw->abort = 1u << rx_dma_chan;

    // Stop the state machine...
    pio_sm_set_enabled(rx_pio, rx_sm, false);

    // Clear SM IRQ (irq0 rel)
    pio_interrupt_clear(pio0, rx_sm + 0);
    //irq_clear(PIO0_IRQ_0)

    // Get packet size....
    length = (uint32_t)rx_dma_chan_hw->write_addr - (uint32_t)rx_frame;

    // Wait for abort to complete
    while (dma_hw->abort & (1ul << rx_dma_chan)) tight_loop_contents();

    // Reconfigure and re-start DMA
    dma_channel_acknowledge_irq0(rx_dma_chan);
    irq_clear(DMA_IRQ_0);   
    dma_channel_hw_addr(rx_dma_chan)->write_addr = (uintptr_t)rx_frame;
    dma_channel_hw_addr(rx_dma_chan)->al1_transfer_count_trig = RX_DMA_LENGTH;
    dma_channel_set_irq0_enabled(rx_dma_chan, true);

    // Start SM (jmp to offset)
    pio_sm_restart(rx_pio, rx_sm);
    pio_sm_clkdiv_restart(rx_pio, rx_sm);
    pio_sm_exec(rx_pio, rx_sm, pio_encode_jmp(rx_offset));
    pio_sm_set_enabled(rx_pio, rx_sm, true);

    printf("PIO_RX_ISR (size=%d)\r\n", length);

    // State machine number impacts the IRQ...
//    pio_interrupt_clear(pio0, 0);
//    pio_interrupt_clear(pio0, 1);
//    irq_clear(PIO0_IRQ_0);
//    flag = 1;
}

//
// We should only get a dma IRQ now if we have overrun the size limit, however
// there is the possiblity that the SM finishes while we are in here and
// pre-empts us ... 
// disable_irqs(); set PIO irq flags; enable_irqs();
//
// Does that fix the problem?
//
void dma_isr() {
    // Is it easier to just trigger a PIO irq?
    // Either by setting the flag manually or by executing a
    // command on the SM? Need to think.
    dma_channel_hw_t *hw = dma_channel_hw_addr(rx_dma_chan);

    dma_channel_acknowledge_irq0(rx_dma_chan);

    uint32_t bytes = (uint32_t)hw->write_addr - (uint32_t)rx_frame;
    printf("DMA OVERRUN bytes=%d\r\n", bytes);
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


    // ------------------------------------
    // Setup the MDIO system and then configure the device so that
    // it autonegotiates and advertises all capabilities. The hardware
    // straps are a little tempermental so it's best to set the values
    // we want in the registers and then do a software reset.

    mdio_init(18, 19);  

    enum {
        SOFT_RESET = (1 << 15),
        SPEED_100 = (1 << 13),
        AUTONEG_EN = (1 << 12),
        AUTONEG_RESTART = (1 << 9),
        FULL_DUPLEX = (1 << 8),
    } basic_config;

    enum {
        FD100 = (1 << 8),
        HD100 = (1 << 7),       // not exactly sure this is what the datasheet means
        FD10 = (1 << 6),
        HD10 = (1 << 5),
        SELECTOR = (0b00001),
    } autoneg_adv;

    enum {
        RESERVED = (1 << 14),
        ALL_CAPABLE = (0b111 << 5),
        ADDRESS1 = 0x1,
    } special_modes;

    uint32_t    rc;
    uint        vendor, model, revision;

    // First lets identify the PHY we are connected to...
    rc = mmio_read(1, MMIO_PHY_ID2);
    vendor = rc >> 10;
    model = rc >> 4 & 0b111111;
    revision = rc & 0b1111;

    printf("Vendor: %d, Model: %d, Revision: %d\r\n", vendor, model, revision);

    // For the LAN8720 we want to force the mode by setting the SPECIAL_MODES
    // register and then soft resetting ... this will get us full autoneg.
    mmio_write(1, MMIO_SPECIAL_MODES, RESERVED|ALL_CAPABLE|ADDRESS1);
    mmio_write(1, MMIO_BASIC_CONTROL, SOFT_RESET); 

    // We don't seem to need any delay after a soft reset, but probably worth
    // doing anyway, just in case...
    sleep_us(50);

    // Not needed, just so we can see the values (remove)
    mmio_read(1, MMIO_BASIC_CONTROL);
    mmio_read(1, MMIO_AUTONEG_ADV);

    for(int i=0; i < 10; i++) {
        // Get status...
        rc = mmio_read(1, MMIO_BASIC_STATUS);
        printf("Status Value -- %08x\r\n", rc);
        sleep_ms(200);
    }


    sleep_ms(500);


    mac_tx_init(13, 15);     // tx0=13, tx1=14, txen=15

    //
    // Now the RX side
    //
    rx_offset = pio_add_program(rx_pio, &macrx_program);
    rx_sm = pio_claim_unused_sm(rx_pio, true);



    // this one doesn't start it...
    macrx_program_init(rx_pio, rx_sm, rx_offset, 26, 28);    // rx0=26, rx1=27, csr=28


    // We have a DMA complete interrupt which will signal if we overrun a buffer
    // Normally it won't complete, the PIO interrupt is the signal the packet
    // is done, and we abort DMA at that point.
    //
    // If DMA overruns then we need to stop and restart the PIO.
    // If PIO finishes, we just need to lower the IRQ it will be waiting on.

    dma_channel_config rx_dma_channel_config;

    rx_dma_chan = dma_claim_unused_channel(true);
    rx_dma_channel_config = dma_channel_get_default_config(rx_dma_chan);
    rx_dma_chan_hw = dma_channel_hw_addr(rx_dma_chan);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_irqn_set_channel_enabled(0, rx_dma_chan, true);



        
    channel_config_set_read_increment(&rx_dma_channel_config, false);
    channel_config_set_write_increment(&rx_dma_channel_config, true);
    channel_config_set_dreq(&rx_dma_channel_config, pio_get_dreq(rx_pio, rx_sm, false));
    channel_config_set_transfer_data_size(&rx_dma_channel_config, DMA_SIZE_8);

    dma_channel_configure(
        rx_dma_chan, &rx_dma_channel_config,
        rx_frame,
        ((uint8_t*)&rx_pio->rxf[rx_sm]) + 3,
        RX_DMA_LENGTH,
        false
    );

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_rx_isr);
    irq_set_enabled(PIO0_IRQ_0, true);


    dma_channel_start(rx_dma_chan);
    pio_sm_set_enabled(rx_pio, rx_sm, true);

    while(1);

    dma_channel_hw_t *hw = dma_channel_hw_addr(rx_dma_chan);
    while(1) {
        uint64_t now = time_us_64();
        while(!flag) {
            if (time_us_64() > now + 3000000) {
                int pc = pio_sm_get_pc(rx_pio, rx_sm);
                printf("No data in 3s ... offset=%d pc=%d\r\n", rx_offset, pc);
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
        pio_sm_set_enabled(rx_pio, rx_sm, false);

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
            ((uint8_t*)&rx_pio->rxf[rx_sm]) + 3,
            RX_DMA_LENGTH,
            false
        );
        dma_sniffer_enable(rx_dma_chan, 0x1, true);
        
        dma_hw->sniff_data = 0xffffffff;

        dma_channel_start(rx_dma_chan);
        macrx_program_init(rx_pio, rx_sm, rx_offset, 26, 28);    // rx0=26, rx1=27, csr=28

        flag = 0;
        pio_sm_set_enabled(rx_pio, rx_sm, true);
    }

    while(1);

    sleep_ms(2000);

    for(int i=0; i < 32; i++) {
        printf("At location: %02x --> %02x\r\n", i, rx_frame[i]);
    }

    printf("Offser is %d\r\n", rx_offset);
    uint8_t pc = pio_sm_get_pc(rx_pio, rx_sm);
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

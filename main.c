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
#include "mactx.pio.h"

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


PIO                 rx_pio = pio0;          // Which PIO are we running on for RX
uint                rx_sm;                 // State machine for RX
uint                rx_offset;             // Start of the RX code

dma_channel_hw_t    *rx_dma_chan_hw;
int                 rx_dma_chan;
uint8_t             rx_frame[2048];

#define RX_DMA_LENGTH         1600

PIO                 tx_pio = pio0;
uint                tx_sm;
uint                tx_offset;

dma_channel_hw_t    *tx_dma_chan_hw;
int                 tx_dma_chan;


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

static uint32_t g_grc_table[] =
{
    0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
    0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
    0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
    0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
};

uint32_t pkt_generate_fcs( uint8_t* data, int length )
{
    uint32_t crc = 0;
    for( uint32_t i = 0 ; i < length ; i++ )
    {
        crc = (crc >> 4) ^ g_grc_table[ ( crc ^ ( data[ i ] >> 0 ) ) & 0x0F ];
        crc = (crc >> 4) ^ g_grc_table[ ( crc ^ ( data[ i ] >> 4 ) ) & 0x0F ];
    }
    return( crc );
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

    //
    // Setup the TX state machine
    //
    tx_offset = pio_add_program(tx_pio, &mactx_program);
    tx_sm = pio_claim_unused_sm(tx_pio, true);

    mactx_program_init(tx_pio, tx_sm, tx_offset, 13, 15);     // tx0=13, tx1=14, txen=15
    pio_sm_set_enabled(tx_pio, tx_sm, true);


    uint8_t packet[] = {
        0x00, 0x00, 0x00, 0x00,             // FOr number of dibits
        0x00, 0x00, 0x00, 0x00,             // For number of padding bytes

        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5,     // preamble

        0x00, 0x01, 0x02, 0x03, 0x04, 0x05,           // destination
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15,           // source
        0x08, 0x00,                                   // ethernet type
        0x45, 0x00,                                   // ip & flags
        0x00, 0x54,                                   // total length (84)
        0x00, 0x00,
        0x40, 0x00,                                     // fragment offset?
        0x40, 0x01,                                 // ttl64, icmp
        0x2f, 0xcb,                                 // header checksum
        0x0a, 0x37, 0x01, 0xfe,                     // source address
        0x0a, 0x37, 0x01, 0xfd,                     // destination
        0x08, 0x00, 0x76, 0x22,                     // ping, zero, checksum
        0x00, 0x06, 0x00, 0x01,                     // id seqency
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,     // timestamp

        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x00, 0x00, 0x00, 0x00,                         // spaxce for checksum
    };

    // First build the number of dibits and padding
    int dibits = (sizeof(packet) - 8) * 4;
    int extra = (sizeof(packet) - 8) % 4;
    int padding = 0;
    if (extra) {
        padding = 4 - extra;
    }

    // Now work out how many transfers...
    int xfers = sizeof(packet)/4;
    if (extra) {
        xfers++;
    }

    // Update the packet with the data...
    uint32_t *p = (uint32_t *)&packet;
    p[0] = dibits;
    p[1] = padding;

    // Now work out the fcs ... this is from the start of the real packet, but not including the
    // fcs bytes
    int fcs_len = sizeof(packet) - (8 + 8 + 4);     // remove control words, preamble, and fcs
    uint32_t fcs = pkt_generate_fcs(&packet[16], fcs_len);

    int fcs_pos = sizeof(packet) - 4;

    printf("fcs=%08x\r\n", fcs);
    packet[fcs_pos++] = (uint8_t)(fcs >> 0);
    packet[fcs_pos++] = (uint8_t)(fcs >> 8);
    packet[fcs_pos++] = (uint8_t)(fcs >> 16);
    packet[fcs_pos++] = (uint8_t)(fcs >> 24);


    dma_channel_config tx_dma_channel_config;
    tx_dma_chan = dma_claim_unused_channel(true);
    tx_dma_channel_config = dma_channel_get_default_config(tx_dma_chan);
    tx_dma_chan_hw = dma_channel_hw_addr(tx_dma_chan);
        
    channel_config_set_read_increment(&tx_dma_channel_config, true);
    channel_config_set_write_increment(&tx_dma_channel_config, false);
    channel_config_set_dreq(&tx_dma_channel_config, pio_get_dreq(tx_pio, tx_sm, true));
    channel_config_set_transfer_data_size(&tx_dma_channel_config, DMA_SIZE_32);

    while(1) {
        dma_channel_configure(
            tx_dma_chan, &tx_dma_channel_config,
            &rx_pio->txf[tx_sm],
            packet,
            xfers,
            false
        );

        // Now send the sizes to the state machine
        //pio_sm_put_blocking(tx_pio, tx_sm, len * 4);      // how many bit pairs
        //pio_sm_put_blocking(tx_pio, tx_sm, padding);

        // Now trigger the dma and wait for it to complete....
        dma_channel_start(tx_dma_chan);

        while(dma_channel_is_busy(tx_dma_chan)) {
            printf("Busy\r\n");
            sleep_ms(200);

        }
        printf("Not busy any more\r\n");
        // Check sm pc...
        int pc = pio_sm_get_pc(tx_pio, tx_sm);
        printf("Started at %d, now at %d\r\n",tx_offset, pc);
        sleep_ms(500);
    }

    while(1);
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

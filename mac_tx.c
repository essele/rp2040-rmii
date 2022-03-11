/**
 * @file mac_tx.c
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "debug.h"
#include "mac_tx.pio.h"
#include "mac_tx.h"
#include "pio_utils.h"

static uint tx_offset;          // so we know where the program loaded
pio_program_t *tx_prog;         // so we know which one it was

//
// We're going to do the initialisation of the PIO TX code in here so we need a way to select
// from the different programs needed for different situations...
//
const static struct pio_prog tx_programs[] = {
    { PIO_PROG(mac_tx_100hd) },
    { PIO_PROG(mac_tx_100fd) },
    { PIO_PROG(mac_tx_10hd) },
    { PIO_PROG(mac_tx_10fd) },
};

//
// Load and configure the relevant PIO program depending on what's needed...
//
static inline int mac_tx_load(PIO pio, uint sm, uint pin_tx0, uint pin_txen, uint pin_crs, int speed, int duplex) {
    pio_sm_config   c;

    int index = (speed == 100) ? duplex : 2 + duplex;
    debug_printf("Loading tx_pio for speed=%d duplex=%d\r\n", speed, duplex);
    struct pio_prog *prog = (struct pio_prog *)&tx_programs[index];

    tx_prog = (pio_program_t *)prog->program;
    tx_offset = pio_add_program(pio, tx_prog);
    c = prog->config_func(tx_offset);

    // Map the state machine's OUT pin group to the two output pins 
    sm_config_set_out_pins(&c, pin_tx0, 2);
    sm_config_set_set_pins(&c, pin_tx0, 2);
    sm_config_set_jmp_pin(&c, pin_crs);
    // txen is a side set pin...
    sm_config_set_sideset_pins(&c, pin_txen);
    // Set this pin's GPIO function (connect PIO to the pad)
    pio_gpio_init(pio, pin_tx0);
    pio_gpio_init(pio, pin_tx0+1);
    pio_gpio_init(pio, pin_txen);
    // Set the pin direction to output at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, pin_tx0, 2, true);      // output
    pio_sm_set_consecutive_pindirs(pio, sm, pin_txen, 1, true);     // output
    // Set direction, autopull, and shift sizes
    sm_config_set_out_shift(&c, true, true, 32);      // shift right, autopull, 32 bits
    // We want to be able to check when the FIFO has anything in it
    // So TX < 1 means STATUS will be all 1's when the fifo is empty
    sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    // Load our configuration, and get ready to start...
    pio_sm_init(pio, sm, tx_offset, &c);
}

static inline void mac_tx_unload(PIO pio, uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_remove_program(pio, tx_prog, tx_offset);
}

static uint                tx_pin_tx0;
static uint                tx_pin_txen;
static uint                tx_pin_crs;          // for carrier detection

static PIO                 tx_pio = pio0;
static uint                tx_sm;
static uint                tx_offset;

static dma_channel_hw_t    *tx_dma_chan_hw;
static int                 tx_dma_chan;         // used for main send
static int                 tx_copy_chan;        // used for copying data around

//
// We don't seem to be able to avoid copying tx packets around a bit, we
// can't use the checksum DMA hardware since it's used on the receive side
// (where it's more important) so we'll have to troll through each byte
// anyway.
//
// My testing shows we're better off using memcpy or DMA to do the transfer
// and then use the checksum code below, rather than a byte by byte checksum
// and copy. And if we use DMA then we can get going on the checksum while
// the transfer is happening. Saves a bit of time.
//
// Because we ideally need to DMA the whole packet to the tx engine in one
// go, we bring it all into one place ... horrible I know, but I can't work
// out a better way for the moment.
//
#define MAX_ETHERNET_BYTES      18 + 1500 + 4   // hdr, payload, crc

struct outgoing_t {
    uint32_t    dibits;                         // How many dibits are going
    uint8_t     preamble[8];                    // preamble
    uint8_t     data[MAX_ETHERNET_BYTES];       // hdr, payload, fcs
};

//struct outgoing_t __scratch_x("sram4_lee") outbufs[2];
struct outgoing_t outbufs[2];
struct outgoing_t *outgoing;
uint              next_outgoing = 0;

// Don't declare this as const, it's probably quicker being
// in memory rather than flash...
static uint32_t g_grc_table[] =
{
    0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
    0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
    0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
    0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
};

/**
 * @brief Generate an ethernet frame check checksum
 * 
 * This is a compromise between lookup tables and speed, we can roughly
 * double the speed by using an 4K or 8K lookup table, but we only actually
 * use this on sending packets so it doesn't feel like it's worth the efforts.
 * 
 * @param data 
 * @param length 
 * @param current_crc
 * @return uint32_t 
 */
static inline uint32_t pkt_generate_fcs( uint8_t* data, int length, uint32_t current_crc )
{
    uint32_t crc = current_crc;
    for( uint32_t i = 0 ; i < length ; i++ )
    {
        crc = (crc >> 4) ^ g_grc_table[ ( crc ^ ( data[ i ] >> 0 ) ) & 0x0F ];
        crc = (crc >> 4) ^ g_grc_table[ ( crc ^ ( data[ i ] >> 4 ) ) & 0x0F ];
    }
    return( crc );
}

/**
 * @brief Print useful packet information for transmitted packets (during debugging)
 * 
 * This will output size, fcs and then the first 16 and last 8 bytes in any
 * transmitted packet.
 * 
 * @param cmnt 
 * @param p 
 * @param length 
 * @param fcs 
 */
static void dump_pkt_info(char *cmnt, uint8_t *p, int length, uint32_t fcs) {
    printf("TX_PKT: %s len=%-4.4d fcs=%08x: ", cmnt, length, fcs);
    for(int i=0; i < 16; i++) { printf("%02x ", p[i]); }
    printf("... ");
    for(int i=length-8; i < length; i++) { printf("%02x ", p[i]); }
    printf("\r\n");
}

/**
 * @brief Setup DMA and state machine in reponse to the interface coming up
 * 
 * @param speed 
 */
void mac_tx_up(int speed, int duplex) {
    //
    // Setup and start the TX state machine
    //
    mac_tx_load(tx_pio, tx_sm, tx_pin_tx0, tx_pin_txen, tx_pin_crs, speed, duplex);
    pio_sm_set_enabled(tx_pio, tx_sm, true);
}

/**
 * @brief Tear everything down as the interface has gone down
 * 
 */
void mac_tx_down() {
    // Abort any inflight DMA, stop and remove code from the SM
    dma_channel_abort(tx_dma_chan);
    mac_tx_unload(tx_pio, tx_sm);

    // TODO: we could be in the middle of copying a packet
    // in which case we'll try to trigger the DMA again even
    // if the link is down.
    // Need to think about how we deal with this.
}

void mac_tx_init(uint pin_tx0, uint pin_txen, uint pin_crs) {
    //
    // Keep the pin details
    //
    tx_pin_tx0 = pin_tx0;
    tx_pin_txen = pin_txen;
    tx_pin_crs = pin_crs;

    //
    // Setup the preamble data in the outgoing packet, this will
    // stay constant.
    //
    static uint8_t preamble[8] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5 };
    for (int i=0; i < 2; i++) {
        memcpy(outbufs[i].preamble, preamble, 8);
    }

    //
    // Setup and start the TX state machine
    //
    tx_sm = pio_claim_unused_sm(tx_pio, true);

    //
    // Setup the DMA channel for copying data around, addresses will be
    // populated later.
    //
    dma_channel_config tx_copy_config;
    tx_copy_chan = dma_claim_unused_channel(true);
    tx_copy_config = dma_channel_get_default_config(tx_copy_chan);
    channel_config_set_read_increment(&tx_copy_config, true);
    channel_config_set_write_increment(&tx_copy_config, true);
    channel_config_set_transfer_data_size(&tx_copy_config, DMA_SIZE_8);
    dma_channel_configure(tx_copy_chan, &tx_copy_config, 0, 0, 0, false);

    //
    // Setup the DMA channel for sending data to the state machine...
    //
    dma_channel_config tx_dma_channel_config;
    tx_dma_chan = dma_claim_unused_channel(true);
    tx_dma_channel_config = dma_channel_get_default_config(tx_dma_chan);
    tx_dma_chan_hw = dma_channel_hw_addr(tx_dma_chan);
        
    channel_config_set_read_increment(&tx_dma_channel_config, true);
    channel_config_set_write_increment(&tx_dma_channel_config, false);
    channel_config_set_dreq(&tx_dma_channel_config, pio_get_dreq(tx_pio, tx_sm, true));
    channel_config_set_transfer_data_size(&tx_dma_channel_config, DMA_SIZE_32);
    dma_channel_configure(tx_dma_chan, &tx_dma_channel_config, &tx_pio->txf[tx_sm], 0, 0, false);
}

// --------------------------------------------------------------------------------
// We have two different send functions, once for just sending a block of uint8_t
// data, and another one that processes LWIP pbuf's. There's quite a bit of
// duplicate code to worry about!
// --------------------------------------------------------------------------------

#ifdef RMII_USE_LWIP

//#include "lwip/def.h"

void mac_tx_send_pbuf(struct pbuf *p) {
    uint32_t fcs = 0;
    struct pbuf *q;

    // If the state machine isn't enabled then the link is down
    // (Note this could change while we're in here!)
    if (!(tx_pio->ctrl & (1 << tx_sm))) {
        debug_printf("Attempted send while interface down, discarding\r\n");
        return;
    }

    // We need to copy the packet to the next outgoing buffer (double buffered)
    // so we don't interfere with the currently transmitting one...
    outgoing = &outbufs[next_outgoing];
    next_outgoing ^= 1;                 // toggle
    uint8_t *out = outgoing->data;

    // We won't know the size until we try to process it...
    uint length = 0;
    for(q=p; q != NULL; q=q->next) {
        // Make the the previous DMA has finished before we start a new one...
        while(dma_channel_is_busy(tx_copy_chan)) tight_loop_contents;

        if (length + q->len > MAX_ETHERNET_BYTES) {
            debug_printf("Attempted send of large packet (%d bytes), discarding\r\n", length);
            return;
        }

        // For each pbuf ... DMA the data over so we can start on the fcs...
        dma_channel_set_read_addr(tx_copy_chan, q->payload, false);
        dma_channel_set_write_addr(tx_copy_chan, out, false);
        dma_channel_set_trans_count(tx_copy_chan, q->len, true);
        fcs = pkt_generate_fcs(q->payload, q->len, fcs);
        out += q->len;
        length += q->len;
    }
    // Deal with small packets... TODO
    if (length < 60) {
        memset(out, 0x00, 60-length);
        fcs = pkt_generate_fcs(out, 60-length, fcs);
        length = 60;
    }
    // Fill in the dibits and work out how many 32bit words it will be
    uint32_t data_bytes = sizeof(outgoing->preamble) + length + 4;   // preamble + data + fcs
    uint dma_size = (data_bytes / 4) + 1;                           // data + length control words

    if (data_bytes % 4) {
        dma_size++;
    }
    outgoing->dibits = (data_bytes * 4) - 1;                         // -1 for the x-- loop

    // Fill in the FCS values
    char *ptr = &outgoing->data[length];
    *ptr++ = (uint8_t)(fcs >> 0);
    *ptr++ = (uint8_t)(fcs >> 8);
    *ptr++ = (uint8_t)(fcs >> 16);
    *ptr++ = (uint8_t)(fcs >> 24);

    // Now wait for (it will be done) the copy DMA to complete...
    while(dma_channel_is_busy(tx_copy_chan)) tight_loop_contents;

    // Now check to ensure the previous packet send has completed...
    while(dma_channel_is_busy(tx_dma_chan)) tight_loop_contents;

    // TODO: potential of the link going down before we get here
    // so SM will be gone, we shouldn't trigger in that case a it
    // will never complete.

#if RMII_DEBUG && RMII_DEBUG_PKT_TX
    dump_pkt_info("OK  ", outgoing->data, length+4, fcs);
#endif

    // Now start the main dma...
    dma_channel_set_read_addr(tx_dma_chan, outgoing, false);
    dma_channel_set_trans_count(tx_dma_chan, dma_size, true);
}

#endif // RMII_USE_LWIP

/**
 * @brief Send an ethernet frame (without preamble or fcs, this will add them)
 * 
 * Once the previous send is complete, this copies the data into the outgoing 
 * buffer, calculates the FCS, and sends to the PIO state machine.
 * @param data 
 * @param length 
 */
void mac_tx_send(uint8_t *data, uint length) {
    uint32_t fcs = 0;

    // If the state machine isn't enabled then the link is down
    // (Note this could change while we're in here!)
    if (!(tx_pio->ctrl & (1 << tx_sm))) {
        debug_printf("Attempted send while interface down, discarding\r\n");
        return;
    }

    // If we try to send a packet that's too big just discard it.
    if (length > MAX_ETHERNET_BYTES) {
        debug_printf("Attempted send of large packet (%d bytes), discarding\r\n", length);
        return;
    }
    // We need to copy the packet to the next outgoing buffer (double buffered)
    // so we don't interfere with the currently transmitting one...
    outgoing = &outbufs[next_outgoing];
    next_outgoing ^= 1;                 // toggle

    // DMA the data over so we can start on the fcs...
    dma_channel_set_read_addr(tx_copy_chan, data, false);
    dma_channel_set_write_addr(tx_copy_chan, outgoing->data, false);
    dma_channel_set_trans_count(tx_copy_chan, length, true);

    // If the packet is less than 60 bytes we need to expand it, this is
    // awkward because the source data (where we normally do the fcs) isn't
    // complete, so we have to fcs on the destination which means waiting for
    // the transfer to complete ... horrible, but should be rare.
    //
    // The normal case means the fcs on the source data can start in parallel
    // with the copy, this saves quite a few cycles.
    if (length < 60) length = 60;
    fcs = pkt_generate_fcs(data, length, fcs);

    // Fill in the dibits and work out how many 32bit words it will be
    uint32_t data_bytes = sizeof(outgoing->preamble) + length + 4;   // preamble + data + fcs
    uint dma_size = (data_bytes / 4) + 1;                           // data + length control words

    if (data_bytes % 4) {
        dma_size++;
    }
    outgoing->dibits = (data_bytes * 4) - 1;                         // -1 for the x-- loop

    // Fill in the FCS values
    char *ptr = &outgoing->data[length];
    *ptr++ = (uint8_t)(fcs >> 0);
    *ptr++ = (uint8_t)(fcs >> 8);
    *ptr++ = (uint8_t)(fcs >> 16);
    *ptr++ = (uint8_t)(fcs >> 24);

    // Now wait for (it will be done) the copy DMA to complete...
    while(dma_channel_is_busy(tx_copy_chan)) tight_loop_contents;

    // Now check to ensure the previous packet send has completed...
    while(dma_channel_is_busy(tx_dma_chan)) tight_loop_contents;

    // TODO: potential of the link going down before we get here
    // so SM will be gone, we shouldn't trigger in that case a it
    // will never complete.

    // Now start the main dma...
    dma_channel_set_read_addr(tx_dma_chan, outgoing, false);
    dma_channel_set_trans_count(tx_dma_chan, dma_size, true);
}

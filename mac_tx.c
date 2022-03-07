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
#include "mac_tx.pio.h"

#include "mac_tx.h"

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

    //tx_dma_chan_hw->al1_ctrl |= (1 << 1);           // High priority!

}


void mac_tx_test() {
        static uint8_t packet[1200] = {
//        0x00, 0x00, 0x00, 0x00,             // FOr number of dibits
//        0x00, 0x00, 0x00, 0x00,             // For number of padding bytes

//        0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5,     // preamble

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
//        0x00, 0x00, 0x00, 0x00,                         // spaxce for checksum
    };

    //printf("Sending ... ");
    mac_tx_send(packet, sizeof(packet));
}

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
        printf("SM down, discarding\r\n");
        return;
    }

    // If we try to send a packet that's too big just discard it.
    if (length > MAX_ETHERNET_BYTES) {
        printf("TX packet too big, discarding\r\n");
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
//    if (0 && length < 60) {
//        while (length < 60) outgoing.data[length++] = 0;
//        while(dma_channel_is_busy(tx_copy_chan)) tight_loop_contents;
//        fcs = pkt_generate_fcs(outgoing.data, length, fcs);
//    } else {
        fcs = pkt_generate_fcs(data, length, fcs);
//    }

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

    int outlen = outgoing->dibits/4;
    uint8_t *p = (uint8_t *)outgoing;

    // Now start the main dma...
    dma_channel_set_read_addr(tx_dma_chan, outgoing, false);
    dma_channel_set_trans_count(tx_dma_chan, dma_size, true);
//    while(dma_channel_is_busy(tx_dma_chan)) tight_loop_contents;

}

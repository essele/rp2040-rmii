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
#define MAX_ETHERNET_BYTES      18 + 1500 + 2  // premable, hdr, payload, crc

struct {
    uint32_t    dibits;                         // How many dibits are going
    uint32_t    padding;                        // How many bytes of padding
    uint8_t     preamble[8];                     // preamble
    uint8_t     data[MAX_ETHERNET_BYTES];       // hdr, payload, fcs
} outgoing;


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
uint32_t pkt_generate_fcs( uint8_t* data, int length, uint32_t current_crc )
{
    uint32_t crc = current_crc;
    for( uint32_t i = 0 ; i < length ; i++ )
    {
        crc = (crc >> 4) ^ g_grc_table[ ( crc ^ ( data[ i ] >> 0 ) ) & 0x0F ];
        crc = (crc >> 4) ^ g_grc_table[ ( crc ^ ( data[ i ] >> 4 ) ) & 0x0F ];
    }
    return( crc );
}


void mac_tx_init(uint pin_tx0, uint pin_txen) {
    //
    // Setup the preamble data in the outgoing packet, this will
    // stay constant.
    //
    static uint8_t preamble[8] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0xd5 };
    memcpy(outgoing.preamble, preamble, 8);

    //
    // Setup and start the TX state machine
    //
    tx_offset = pio_add_program(tx_pio, &mac_tx_program);
    tx_sm = pio_claim_unused_sm(tx_pio, true);

    mac_tx_program_init(tx_pio, tx_sm, tx_offset, pin_tx0, pin_txen);    
    pio_sm_set_enabled(tx_pio, tx_sm, true);

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


    return;


}


void mac_tx_test() {
        uint8_t packet[] = {
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

    printf("Sending ... ");
    mac_tx_send(packet, sizeof(packet));
}

/**
 * @brief Remove everything TX so we can reconfigure
 * 
 * This stops and removes the DMA channels, stops and unloads the
 * state machine. This is so that we can reload it with different
 * versions as we re-negotiate 10/100/half/full.
 * 
 * Anything in flight will be aborted .. there are no IRQ's on the
 * TX side, so this shouldn't have any side effects.
 */
void mac_tx_teardown() {
    // TODO
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

    // DMA the data over so we can start on the fcs...
    dma_channel_set_read_addr(tx_copy_chan, data, false);
    dma_channel_set_write_addr(tx_copy_chan, outgoing.data, false);
    dma_channel_set_trans_count(tx_copy_chan, length, true);

    // Now (in parallel) work out the fcs... use the source so
    // we can ultimately support split packets...
    fcs = pkt_generate_fcs(data, length, fcs);

    // Fill in the dibits and padding
    uint32_t data_bytes = sizeof(outgoing.preamble) + length + 4;   // preamble + data + fcs
    uint extra = data_bytes % 4;
    uint32_t padding = 0;
    uint dma_size = (data_bytes / 4) + 2;       // data + 2 x control words
    if (extra) {
        padding = 4 - extra;
        dma_size++;
    }
    outgoing.dibits = data_bytes * 4;
    outgoing.padding = padding;

    // Fill in the FCS values
    char *ptr = &outgoing.data[length];
    *ptr++ = (uint8_t)(fcs >> 0);
    *ptr++ = (uint8_t)(fcs >> 8);
    *ptr++ = (uint8_t)(fcs >> 16);
    *ptr++ = (uint8_t)(fcs >> 24);

    // Now wait for (make sure) the copy DMA to complete...
    while(dma_channel_is_busy(tx_copy_chan)) tight_loop_contents;

    // Now check to ensure the previous packet send has completed...
    while(dma_channel_is_busy(tx_dma_chan)) tight_loop_contents;

    // Now start the main dma...
    dma_channel_set_read_addr(tx_dma_chan, &outgoing, false);
    dma_channel_set_trans_count(tx_dma_chan, dma_size, true);
}

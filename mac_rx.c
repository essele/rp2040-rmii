/**
 * @file mac_rx.c
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-04
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
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "mac_rx.pio.h"

uint                rx_pin_rx0;
uint                rx_pin_crs;

PIO                 rx_pio = pio0;          // Which PIO are we running on for RX
uint                rx_sm;                 // State machine for RX
//uint                rx_offset;             // Start of the RX code

dma_channel_hw_t    *rx_dma_chan_hw;
int                 rx_dma_chan;

volatile int flag = 0;

#define RX_FRAME_COUNT  8
//#define RX_MAX_BYTES    1446              // for testing large packet handling
#define RX_MAX_BYTES    1600
#define ETHER_CHECKSUM  0xc704dd7b


// We need to keep the identification of free frames as quick as possible
// since it's done in the ISR. So we'll keep a singly linked list, we can
// easily add and remove from the front.
struct rx_frame {
    struct rx_frame     *next;
    int                 length;
    int                 checksum;
    uint8_t             data[RX_MAX_BYTES];
};

struct rx_frame rx_frames[RX_FRAME_COUNT];
struct rx_frame *rx_free_list;                  // list of free-to-use frames
struct rx_frame *rx_current;                    // the one being used

void rx_frame_init() {
    int i;

    for (i=0; i < RX_FRAME_COUNT-1; i++) {
        rx_frames[i].next = &rx_frames[i+1];
    }
    rx_frames[i].next = (struct rx_frame *)NULL;
    rx_free_list = &rx_frames[0];

    rx_current = (struct rx_frame *)NULL;
}

/**
 * @brief Picks a free frame
 * 
 * Finds a free frame just by popping one off the free list.
 * If the list is empty then you will get a NULL return value.
 *
 * This should only be called by the ISR so no need to disable
 * IRQ's as we'll have already pre-empted the main thread.
 * 
 * @return struct rx_frame *
 */
struct rx_frame *rx_get_free_frame() {
    struct rx_frame *rc = rx_free_list;

    if (rc) {
        rx_free_list = rc->next;
        rc->next = 0;
    }
    return rc;
}

/**
 * @brief Return a frame to the free pool
 * 
 * This must be done with IRQ's disabled so that we avoid a potential race
 * condition with the ISR trying to get a free frame.
 * 
 * @param frame 
 */
void rx_add_to_free_list(struct rx_frame *frame) {
    uint32_t irq_save = save_and_disable_interrupts();
    frame->next = rx_free_list;
    rx_free_list = frame;
    restore_interrupts(irq_save);
}
/**
 * @brief As per rx_add_to_free_list but for use in the ISR
 * 
 * @param frame 
 */
static inline void rx_add_to_free_list_noirq(struct rx_frame *frame) {
    frame->next = rx_free_list;
    rx_free_list = frame;
}



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
// NOTE: if a packet is only just over the maximum it's possible this
// isr will get called before the DMA one, in which case we will know
// because of the length. The DMR IRQ will be cleared so we don't need
// to worry about that.
//
//
void pio_rx_isr() {
    int         overrun = 0;
    uint32_t    checksum;

    // First disable the DMA irq from happening when we abort...
    dma_channel_set_irq0_enabled(rx_dma_chan, false);

    // Now abort the DMA channel... (this won't wait for outstanding xfers)
    dma_hw->abort = 1u << rx_dma_chan;

    // Wait for abort to complete (I've never seen any looping here, it's already done)
    while (dma_hw->ch[rx_dma_chan].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) tight_loop_contents();

    // Immediately find a new frame for the next incoming packet, we can process this one
    // after everything is back up and running... if we don't have a free one then we just
    // reuse this one. Make sure rx_current is updated to the new packet.
    struct rx_frame *received = rx_current;
    struct rx_frame *new = rx_get_free_frame();
    if (!new) {
        overrun = 1;
    } else {
        rx_current = new;
        received->length = (uint32_t)rx_dma_chan_hw->write_addr - (uint32_t)received->data;
        received->checksum = dma_hw->sniff_data;
    }

    // Now we can get DMA and the state machine back up and running...
    // Reconfigure and re-start DMA
    dma_channel_acknowledge_irq0(rx_dma_chan);
    dma_channel_hw_addr(rx_dma_chan)->al2_write_addr_trig = (uintptr_t)rx_current->data;
    dma_channel_set_irq0_enabled(rx_dma_chan, true);
    dma_hw->sniff_data = 0xffffffff;

    // Clear any pending DMA IRQ that could have been caused by a perfect match overrun
    irq_clear(DMA_IRQ_0);

    // Restart the state machine by clearing the IRQ
    pio_interrupt_clear(pio0, rx_sm + 0);

    // Now we are nicely running we can do the processing that's less time critical
    if (overrun) {
        printf("BUFFER OVERRUN\r\n");
        // We haven't used the packet, so no need to return it...
        return;
    }
    // Catch the case of a large packet that hasn't tripped the DMA overrun check
    // TODO: this should be a define...
    if (received->length >= 1536) {
        printf("LARGE PACKET\r\n");
        rx_add_to_free_list_noirq(received);
        return;
    }
    if (received->checksum != ETHER_CHECKSUM) {
        printf("CHECKSUM ERROR: %08x (length=%d)\r\n", received->checksum, received->length);
        // We need to return this packet and don't process any further
        rx_add_to_free_list_noirq(received);
        return;
    }
    printf("PIO_RX_ISR (size=%d / addr=%08x / fcs=%08x):", received->length, received, received->checksum);
    for (int i=0; i < 16; i++) {
        printf(" %02x", received->data[i]);
    }
    printf("\r\n");
    // Plonk the packet back for now...
    rx_add_to_free_list_noirq(received);
}

//
// We should only get a dma IRQ now if we have overrun the size limit, however
// there is the possiblity that the SM finishes while we are in here and
// pre-empts us ... we should be able to tell that by looking if DMA is still
// busy.
//
void dma_isr() {
    uint32_t save = save_and_disable_interrupts();
    int type = 0;

    if (dma_channel_is_busy(rx_dma_chan)) {
        // This was an overrun, but the SM also finished and that IRQ has been processed and
        // DMA would have been restarted.
        // This is probably a bad thing since we may have stuff in the FIFO and DMA
        // will start transferring...
        // TODO: I've yet to see this happen ...
        dma_channel_acknowledge_irq0(rx_dma_chan);
    } else {
        // This is just a straightforward overrun of a packet the SM will be running or
        // blocked trying to put stuff in the FIFO and DMA is stopped, so we need to reset
        // both things... the SM could also have finished and raised an IRQ while we are
        // in here, so we need to clear that to.
        pio_sm_set_enabled(rx_pio, rx_sm, false);       // stop the SM
        pio_interrupt_clear(pio0, rx_sm + 0);           // not sure if restart does this as well?
        pio_sm_restart(rx_pio, rx_sm);                  // clear state
        pio_sm_clear_fifos(rx_pio, rx_sm);
        pio_sm_exec(rx_pio, rx_sm, pio_encode_jmp(rx_offset));

        // Now we need to re-establish the DMA and restart on the current packet...
        dma_channel_acknowledge_irq0(rx_dma_chan);
        dma_channel_hw_addr(rx_dma_chan)->al2_write_addr_trig = (uintptr_t)rx_current->data;
        dma_hw->sniff_data = 0xffffffff;

        // Clear any pending PIO irq (if it finished while we were in here)
        irq_clear(PIO0_IRQ_0);

        // And off we go...
        pio_sm_set_enabled(rx_pio, rx_sm, true);
        type = 2;
    }

    restore_interrupts(save);
    printf("DMA OVERRUN IRQ: type = %s\r\n", type == 1 ? "AAARRRGH" : "NORMAL");
}


void mac_rx_up(int speed) {
    mac_rx_load(rx_pio, rx_sm, rx_pin_rx0, rx_pin_crs, speed);     // rx0=26, rx1=27, crs=28, 10MBPS

    // TODO: duplicate code, need to find a nicer way...
    struct rx_frame *first = rx_get_free_frame();
    rx_current = first;

    dma_sniffer_enable(rx_dma_chan, 0x1, true);
    dma_hw->sniff_data = 0xffffffff;
    dma_channel_hw_addr(rx_dma_chan)->al2_write_addr_trig = (uintptr_t)rx_current->data;
    dma_channel_set_irq0_enabled(rx_dma_chan, true);
    pio_sm_set_enabled(rx_pio, rx_sm, true);
}

/**
 * @brief Stop DMA and SM.
 * 
 * We need to cleanly stop anything in flight, so will need to disable
 * IRQs and cleanup nicely.
 */
void mac_rx_down() {
    // Abort any inflight DMA, stop and remove code from the SM
    uint32_t save = save_and_disable_interrupts();

    dma_channel_abort(rx_dma_chan);
    mac_rx_unload(rx_pio, rx_sm);

    // Now clear anuy flags that could be set
    dma_channel_acknowledge_irq0(rx_dma_chan);
    pio_interrupt_clear(pio0, rx_sm + 0);
    irq_clear(DMA_IRQ_0);
    irq_clear(PIO0_IRQ_0);

    // We will have a packet being filled so we'll need to return
    // that to the free pool
    rx_add_to_free_list_noirq(rx_current);
    rx_current = NULL;          // cause an error if we do something stupid!

    restore_interrupts(save);
}


void mac_rx_init(uint pin_rx0, uint pin_crs) {
    // Keep the pins for later...
    rx_pin_rx0 = pin_rx0;
    rx_pin_crs = pin_crs;

    rx_sm = pio_claim_unused_sm(rx_pio, true);


    // We have a DMA complete interrupt which will signal if we overrun a buffer
    // Normally it won't complete, the PIO interrupt is the signal the packet
    // is done, and we abort DMA at that point.
    //
    // If DMA overruns then we need to stop and restart the PIO.
    // If PIO finishes, we just need to lower the IRQ it will be waiting on.

    rx_frame_init();

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

    // Dummy configure, so we only have to change write address later...
    dma_channel_configure(
        rx_dma_chan, &rx_dma_channel_config,
        NULL,
        ((uint8_t*)&rx_pio->rxf[rx_sm]) + 3,
        RX_MAX_BYTES,
        false
    );

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_rx_isr);
    irq_set_enabled(PIO0_IRQ_0, true);
}
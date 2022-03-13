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

#include "debug.h"
#include "mac_rx.pio.h"
#include "mac_rx.h"
#include "pio_utils.h"

static uint tx_offset;          // so we know where the program loaded
pio_program_t *tx_prog;         // so we know which one it was

//
// We're going to do the initialisation of the PIO RX code in here so we need a way to select
// from the different programs needed for different situations...
//
#if (RMII_SYS_MHZ == 150)
const static struct pio_prog rx_programs[] = {
    { PIO_PROG(mac_rx_10_150MHz) },
    { PIO_PROG(mac_rx_100_150MHz) },
};
#else
const static struct pio_prog rx_programs[] = {
    { PIO_PROG(mac_rx_10) },
    { PIO_PROG(mac_rx_100) },
};
#endif

static uint rx_offset;
pio_program_t *rx_prog;

//
// Load and configure the relevant PIO program depending on what's needed...
//
static inline int mac_rx_load(PIO pio, uint sm, int speed) {
    pio_sm_config c;

    int index = (speed == 100) ? 1 : 0;
    struct pio_prog *prog = (struct pio_prog *)&rx_programs[index];

    rx_prog = (pio_program_t *)prog->program;
    rx_offset = pio_add_program(pio, rx_prog);
    c = prog->config_func(rx_offset);

    // Setup the configuration...
    sm_config_set_in_pins(&c, RMII_PIN_RX0);
    sm_config_set_jmp_pin(&c, RMII_PIN_CRS);

    // Connect PIO to the pads (not technically needed for rx, but cleaner)
    pio_gpio_init(pio, RMII_PIN_RX0);
    pio_gpio_init(pio, RMII_PIN_RX1);
    pio_gpio_init(pio, RMII_PIN_CRS);

    // Set the pin direction to input at the PIO
    pio_sm_set_consecutive_pindirs(pio, sm, RMII_PIN_RX0, 2, false);     // input
    pio_sm_set_consecutive_pindirs(pio, sm, RMII_PIN_RX1, 1, false);     // input

    // Set direction, autopull, and shift sizes
    sm_config_set_in_shift(&c, true, true, 32);                      // shift right, autopush, 8 bits

    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);                  // join the RX fifos

    pio_interrupt_clear(pio, sm);
    pio_set_irq0_source_enabled(pio, pis_interrupt0 + sm, true);

    // Load our configuration, and get ready to start...
    pio_sm_init(pio, sm, rx_offset, &c);
}

static inline void mac_rx_unload(PIO pio, uint sm) {
    pio_sm_set_enabled(pio, sm, false);
    pio_interrupt_clear(pio, sm);
    pio_set_irq0_source_enabled(pio, pis_interrupt0 + sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_remove_program(pio, rx_prog, rx_offset);
}




PIO                 rx_pio = pio0;          // Which PIO are we running on for RX
uint                rx_sm;                  // State machine for RX

dma_channel_hw_t    *rx_dma_chan_hw;
int                 rx_dma_chan;

struct rx_stats {
    int32_t     packets;
    int32_t     oob;
    int32_t     fcs;
    int32_t     big;
};

struct rx_stats rxstats;


#define ETHER_CHECKSUM0  0xc704dd7b                 // no trailing zeros
#define ETHER_CHECKSUM1  0x8104c946                 // one trailing zero (-3 on length)
#define ETHER_CHECKSUM2  0x3a7abc72                 // two trailing zeros (-2 on length)
#define ETHER_CHECKSUM3  0x4710bb9c                 // three trailing zeros (-1 on length)

void print_rx_stats() {
    printf("pkts=%d oob=%d fcs=%d big=%d\r\n", rxstats.packets, rxstats.oob, rxstats.fcs, rxstats.big);
}



// We need to keep the identification of free frames as quick as possible
// since it's done in the ISR. So we'll keep a singly linked list, we can
// easily add and remove from the front.
struct rx_frame rx_frames[RX_FRAME_COUNT];
volatile struct rx_frame *rx_free_list;                  // list of free-to-use frames
struct rx_frame *rx_current;                    // the one being used

volatile struct rx_frame *rx_ready_list;                 // a list of packets needing to be processed
volatile struct rx_frame *rx_ready_tail;                 // the last item in the list (so we can add to the end)

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
    struct rx_frame *rc = (struct rx_frame *)rx_free_list;

    if (rc) {
        rx_free_list = (struct rx_frame *)rc->next;
//        rc->next = 0;         // not important when out of a list
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
    frame->next = (struct rx_frame *)rx_free_list;
    rx_free_list = frame;
    restore_interrupts(irq_save);
}
/**
 * @brief As per rx_add_to_free_list but for use in the ISR
 * 
 * @param frame 
 */
static inline void rx_add_to_free_list_noirq(struct rx_frame *frame) {
    frame->next = (struct rx_frame *)rx_free_list;
    rx_free_list = frame;
}

/**
 * @brief Adds a frame to the tail of the ready list
 * 
 * @param frame 
 */
static inline void rx_add_to_ready_list_noirq(struct rx_frame *frame) {
    frame->next = 0;

    // First deal with the empty case...
    if (!rx_ready_tail) {
        rx_ready_list = frame;
        rx_ready_tail = frame;
    } else {
        // At least one is in there, so add to the tail
        rx_ready_tail->next = frame;
        rx_ready_tail = frame;
    }
}

/**
 * @brief Return a frame from the ready list
 * 
 * Returns the next frame from the head of the ready list or NULL if there aren't
 * any available. Interrupts are disabled while the list is updated to avoid 
 * conflicts with the ISR which adds them.
 * 
 * @return struct rx_frame* 
 */
struct rx_frame *rx_get_ready_frame() {
    struct rx_frame *rc;
    uint32_t irq_save = save_and_disable_interrupts();
    rc = (struct rx_frame *)rx_ready_list;
    if (rc) {
        rx_ready_list = rx_ready_list->next;
        if (!rx_ready_list) {       // did we get the last one?
            rx_ready_tail = 0;
        }
        //rc->next = 0;             // not important while out of a list
    }
    restore_interrupts(irq_save);
    return(rc);
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
void __time_critical_func(pio_rx_isr)() {
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
    struct rx_frame *received = (struct rx_frame *)rx_current;
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
        rxstats.oob++;
        // We haven't used the packet, so no need to return it...
        return;
    }
    // Process checksum matching and length adjustment...
    if (received->checksum == ETHER_CHECKSUM0) {
    } else if (received->checksum == ETHER_CHECKSUM1) {
        received->length -= 3;
    } else if (received->checksum == ETHER_CHECKSUM2) {
        received->length -= 2;
    } else if (received->checksum == ETHER_CHECKSUM3) {
        received->length--;
    } else {
        rxstats.fcs++;
        debug_printf("RX CHECKSUM ERROR: %08x (length=%d)\r\n", received->checksum, received->length);
#if RMII_DEBUG && RMII_DEBUG_PKT_RX
            dump_pkt_info("RX FCS!", received->data, received->length, received->checksum);
#endif
        // We need to return this packet and don't process any further
        rx_add_to_free_list_noirq(received);
        return;
    }

    // Catch the case of a large packet that hasn't tripped the DMA overrun check
    // TODO: this should be a define...
    if (received->length >= 1536) {
        rxstats.big++;
        rx_add_to_free_list_noirq(received);
        return;
    }

    // Now add the packet to the ready list...
    rx_add_to_ready_list_noirq(received);
    rxstats.packets++;

#if RMII_DEBUG && RMII_DEBUG_PKT_RX
    dump_pkt_info("RX OK  ", received->data, received->length, ETHER_CHECKSUM0);
#endif
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
    mac_rx_load(rx_pio, rx_sm, speed);     // rx0=26, rx1=27, crs=28, 10MBPS

    // TODO: duplicate code, need to find a nicer way...
    struct rx_frame *first = rx_get_free_frame();
    rx_current = first;

    dma_sniffer_enable(rx_dma_chan, 0x1, true);
//    dma_sniffer_set_byte_swap_enabled(true);
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
    channel_config_set_transfer_data_size(&rx_dma_channel_config, DMA_SIZE_32);

    // Dummy configure, so we only have to change write address later...
    dma_channel_configure(
        rx_dma_chan, &rx_dma_channel_config,
        NULL,
        ((uint8_t*)&rx_pio->rxf[rx_sm]),
        RX_MAX_BYTES / 4,
        false
    );
    //rx_dma_chan_hw->al1_ctrl |= (1 << 1);           // High priority!

    irq_set_exclusive_handler(PIO0_IRQ_0, pio_rx_isr);
    irq_set_enabled(PIO0_IRQ_0, true);
}
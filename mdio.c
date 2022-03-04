/**
 * @file mdio.c
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/* --------------------------------------------------------------------
 * MMIO -- routines to support the MMIO function, along with mmio.pio
 *         whcih handles the PIO state machine.
 * --------------------------------------------------------------------
 */
#include <stdio.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/irq.h"
#include "mdio.h"
#include "mdio.pio.h"

#include "mac_tx.h"
#include "mac_rx.h"

static PIO     mdio_pio = pio0;
static int     mdio_sm;
static int     mdio_addr;

#define MDIO_LOW_PRIORITY_IRQ       30              // Avoid the stdio_usb one
#define MDIO_TASK_INTERVAL_US       10000           // 10ms should be fine

static struct {
    int speed;
    char *description;
} modes[8] = {
    { 0,    "Link down" },
    { 10,   "10BASE-T half duplex" },
    { 100,  "100BASE-TX half duplex" },
    { 0,    "Unknown Mode" },
    { 0,    "Unknown Mode" },
    { 10,   "10BASE-T full duplex" },
    { 100,  "100BASE-TX full duplex" },
    { 0,    "Unknown Mode" },
};

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

enum {
    LINK_STATUS = (1 << 2),
    AUTONEG_COMPLETE = (1 << 5),
} basic_status;

enum {
    AUTODONE = (1 << 12),
    SP_RESERVED = (0b0000010 << 5),
    SPEED = (0b111 << 2),
} special_status;


uint16_t mdio_read(int addr, int reg) {
    uint32_t op = (1 << 30) | (2 << 28) | (addr << 23) | (reg << 18) | (0b11 << 16) | 0xffff;
    uint32_t rc;

    pio_sm_put_blocking(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_put_blocking(mdio_pio, mdio_sm, ~op);      // Send the code (inverted because it's pindirs)
    pio_sm_get_blocking(mdio_pio, mdio_sm);           // Read the dummy value
    rc = pio_sm_get_blocking(mdio_pio, mdio_sm);
    return (rc & 0xffff);
}
void mdio_send_read_cmd(int addr, int reg) {
    uint32_t op = (1 << 30) | (2 << 28) | (addr << 23) | (reg << 18) | (0b11 << 16) | 0xffff;

    pio_sm_put(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_put(mdio_pio, mdio_sm, ~op);      // Send the code (inverted because it's pindirs)
}
uint16_t mdio_get_read_result() {
    pio_sm_get(mdio_pio, mdio_sm);           // Read the dummy value
    return pio_sm_get(mdio_pio, mdio_sm) & 0xffff;
}


void mdio_write(int addr, int reg, uint16_t value) {
    uint32_t op = (1 << 30) | (1 << 28) | (addr << 23) | (reg << 18) | (0b10 << 16) | value;
    uint32_t rc;
    pio_sm_put_blocking(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_put_blocking(mdio_pio, mdio_sm, ~op);      // Send the code (inverted because it's pindirs)
    pio_sm_get_blocking(mdio_pio, mdio_sm);           // Read the dummy value
    rc = pio_sm_get_blocking(mdio_pio, mdio_sm);
}


// Do as little as possible in the timer task, so just trigger a low priority
// IRQ to actually do the work.
static int64_t mdio_timer_task(__unused alarm_id_t id, __unused void *user_data) {
    irq_set_pending(MDIO_LOW_PRIORITY_IRQ);
    return MDIO_TASK_INTERVAL_US;
}

// See if we have a response to our previous query and process it. If not just send
// a new one.
uint32_t last_rval = 0;

int link_status = 0;        // 0 = down


static void mdio_isr() {
    uint32_t rval;

    if (pio_sm_get_rx_fifo_level(mdio_pio, mdio_sm) > 1) {
        rval = mdio_get_read_result();
        int x = pio_sm_get_rx_fifo_level(mdio_pio, mdio_sm);
        if (x) {
            printf("Fifo level is %d\r\n", x);
        }
        if (rval != last_rval) {
            printf("Rval changed from %04x to %04x\r\n", last_rval, rval);
            last_rval = rval;

            uint16_t sp = mdio_read(mdio_addr, MDIO_SPECIAL_STATUS);
            printf("SPECIAL=%04x\r\n", sp);
            int mode = (sp & SPEED) >> 2;
            printf("Mode is: %s\r\n", modes[mode].description);

            int new_link_status = !!(rval & LINK_STATUS);
            if (new_link_status != link_status) {
                link_status = new_link_status;
                printf("Link status now: %d\r\n", link_status);

                if (link_status) {
                    mac_tx_up(modes[mode].speed);
                    mac_rx_up(modes[mode].speed);
                } else {
                    mac_tx_down();
                    mac_rx_down();
                }

            }
        }
        x = pio_sm_get_rx_fifo_level(mdio_pio, mdio_sm);
        if (x) {
            printf("B4 send Fifo level is %d\r\n", x);
        }

        mdio_send_read_cmd(mdio_addr, MDIO_BASIC_STATUS);
    }
}

/**
 * @brief Setup and start the mmio PIO state machine
 * 
 * @param pin_mdc 
 * @param pin_mdio 
 * @return int 
 */
int mdio_init(uint pin_mdc, uint pin_mdio) {
    uint offset;

    mdio_sm = pio_claim_unused_sm(mdio_pio, true);
    offset = pio_add_program(mdio_pio, &mdio_program);
    mdio_program_init(mdio_pio, mdio_sm, offset, pin_mdc, pin_mdio);

    // Now pump some null data into the phy to make sure we're not half way through something
    // after a reset.
    pio_sm_clear_fifos(mdio_pio, mdio_sm);
    pio_sm_put_blocking(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_put_blocking(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_get_blocking(mdio_pio, mdio_sm);
    pio_sm_get_blocking(mdio_pio, mdio_sm);

    // TODO ... detect the mdio address
    mdio_addr = 1;

    uint32_t    rc;
    uint        vendor, model, revision;

    // First lets identify the PHY we are connected to...
    rc = mdio_read(mdio_addr, MDIO_PHY_ID2);
    vendor = rc >> 10;
    model = rc >> 4 & 0b111111;
    revision = rc & 0b1111;

    printf("Vendor: %d, Model: %d, Revision: %d\r\n", vendor, model, revision);

    // For the LAN8720 we want to force the mode by setting the SPECIAL_MODES
    // register and then soft resetting ... this will get us full autoneg.
    mdio_write(mdio_addr, MDIO_SPECIAL_MODES, RESERVED|ALL_CAPABLE|ADDRESS1);
    mdio_write(mdio_addr, MDIO_BASIC_CONTROL, SOFT_RESET); 

    // We don't seem to need any delay after a soft reset, but probably worth
    // doing anyway, just in case...
    sleep_us(50);

    // Now setup the ISR and timer
    irq_set_exclusive_handler(MDIO_LOW_PRIORITY_IRQ, mdio_isr);
    irq_set_enabled(MDIO_LOW_PRIORITY_IRQ, true);

    add_alarm_in_us(MDIO_TASK_INTERVAL_US, mdio_timer_task, NULL, true);

    // Send the first read command (which the ISR will respond to)
    mdio_send_read_cmd(mdio_addr, MDIO_BASIC_STATUS);
}




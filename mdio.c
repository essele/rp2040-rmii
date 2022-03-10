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
static int     mdio_type;

#define MDIO_LOW_PRIORITY_IRQ       30              // Avoid the stdio_usb one
#define MDIO_TASK_INTERVAL_US       10000           // 10ms should be fine

#define MDIO_BASIC_CONTROL          0
#define MDIO_BASIC_STATUS           1
#define MDIO_PHY_ID2                3
#define MDIO_AUTONEG_ADV            4
#define MDIO_SPECIAL_MODES          18
#define MDIO_SPECIAL_STATUS         31

static struct {
    int speed;
    int duplex;             // 0 = half, 1 = full   
    char *description;
} modes[8] = {
    { 0,    0,  "Link down" },
    { 10,   0,  "10BASE-T half duplex" },
    { 100,  0,  "100BASE-TX half duplex" },
    { 0,    0,  "Unknown Mode" },
    { 0,    0,  "Unknown Mode" },
    { 10,   1,  "10BASE-T full duplex" },
    { 100,  1,  "100BASE-TX full duplex" },
    { 0,    0,  "Unknown Mode" },
};

//
// Variations in the way we interact with the PHY
//
#define TYPE_BASIC              0
#define TYPE_LAN8720            1

struct phy {
    uint16_t    id2;
    int         type;
    char        *description;
};

// We ignore the lower 4 bits of ID2 since this is a revision number and could
// change with silicon versions.
static struct phy phys[] = {
    { 0xc0f0, TYPE_LAN8720, "Microchip LAN8720" },
    { 0x0000, TYPE_BASIC,   "Blah Blah Phy" },
    { 0x0000, 0, NULL }     // end of list
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
//
// We monitor the link status, and then if that changes we use the special status
// register to work out what's going on (speed/duplex)

static uint32_t last_status = 0;
static int      sm_loaded = 0;

//int link_status = 0;        // 0 = down

/**
 * @brief Internal handler, called by poll or ISR
 * 
 * @return int 
 */
static int mdio_handler() {
    int rc = LINK_NO_CHANGE;

    if (pio_sm_get_rx_fifo_level(mdio_pio, mdio_sm) > 1) {
        int new_status = !!(mdio_get_read_result() & LINK_STATUS);

        if (new_status != last_status) {
            // The link has gone up or down... if down then unload
            if (!new_status) {
                printf("Link DOWN\r\n");
                if (sm_loaded) {
                    mac_tx_down();
                    mac_rx_down();
                    sm_loaded = 0;
                }
                rc = LINK_DOWN;
            } else {
                uint mode = (mdio_read(mdio_addr, MDIO_SPECIAL_STATUS) & SPEED) >> 2;
                int speed = modes[mode].speed;
                int duplex = modes[mode].duplex;
                printf("Link UP, %s\r\n", modes[mode].description);

                if (!sm_loaded) {
                    mac_tx_up(speed, duplex);
                    mac_rx_up(speed);
                    sm_loaded = 1;
                }
                switch(speed + duplex) {
                    case 10:    rc = LINK_UP_10HD; break;
                    case 11:    rc = LINK_UP_10FD; break;
                    case 100:   rc = LINK_UP_100HD; break;
                    case 101:   rc = LINK_UP_100FD; break;
                    default:    rc = LINK_UP_UNKNOWN; break;
                }
            }
            last_status = new_status;
        }
        mdio_send_read_cmd(mdio_addr, MDIO_BASIC_STATUS);
    }
    return rc;
}

/**
 * @brief The ISR just calls the handler and throws away the result
 * 
 * TODO: how do we trigger events?
 */
static void mdio_isr() {
    mdio_handler();
}

/**
 * @brief Polling version of MDIO machine rather than IRQ's
 *
 */
int mdio_poll() {
    static uint64_t last_call = 0;
    uint64_t        now = time_us_64();
    int             rc = LINK_NO_CHANGE;

    if(now > last_call + MDIO_TASK_INTERVAL_US) {
        rc = mdio_handler();
        last_call = now;
    }
    return rc;
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

    // Now see if we can figure out which address the PHY is on, and which vendor and model
    // it is.
    int         i;
    uint32_t    rc;
    uint        vendor, model, revision;
    for (i=0; i < 32; i++) {
        rc = mdio_read(i, MDIO_PHY_ID2);
        if (rc != 0xffff) break;
    }
    if (i == 32) {
        printf("No PHY detected, not starting rmii\r\n");
        return 0;
    }
    mdio_addr = i;

    vendor = rc >> 10;
    model = rc >> 4 & 0b111111;
    revision = rc & 0b1111;

    // Now see if we recognise the phy... ignoring the revision
    struct phy *p = phys;
    while (1) {
        if (p->id2 == 0) {
            printf("PHY@addr=%d: unknown, using basic capabilities (id2=0x%04x)\r\n", mdio_addr, rc);
            mdio_type = TYPE_BASIC;
            break;
        }
        if ((p->id2 & 0xfff0) == (rc & 0xfff0)) {
            printf("PHY@addr=%d: %s (revision %x)\r\n", mdio_addr, p->description, revision);
            mdio_type = p->type;
            break;
        }
        p++;
    }

    // For the LAN8720 we want to force the mode by setting the SPECIAL_MODES
    // register and then soft resetting ... this will get us full autoneg.
    if (mdio_type == TYPE_LAN8720) {
        mdio_write(mdio_addr, MDIO_SPECIAL_MODES, RESERVED|ALL_CAPABLE|ADDRESS1);
        mdio_write(mdio_addr, MDIO_BASIC_CONTROL, SOFT_RESET); 

        // We don't seem to need any delay after a soft reset, but probably worth
        // doing anyway, just in case...
        sleep_us(50);
    }

    // Now setup the ISR and timer
#ifdef MDIO_USE_IRQ
    irq_set_exclusive_handler(MDIO_LOW_PRIORITY_IRQ, mdio_isr);
    irq_set_enabled(MDIO_LOW_PRIORITY_IRQ, true);

    add_alarm_in_us(MDIO_TASK_INTERVAL_US, mdio_timer_task, NULL, true);
#endif

    // Send the first read command (which the ISR or polling system will respond to)
    mdio_send_read_cmd(mdio_addr, MDIO_BASIC_STATUS);
}

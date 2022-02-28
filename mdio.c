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
#include "mdio.h"
#include "mdio.pio.h"

static PIO     mdio_pio = pio0;
static int     mdio_sm;

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
}



uint16_t mmio_read(int addr, int reg) {
    uint32_t op = (1 << 30) | (2 << 28) | (addr << 23) | (reg << 18) | (0b11 << 16) | 0xffff;
    uint32_t rc;

    pio_sm_put_blocking(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_put_blocking(mdio_pio, mdio_sm, ~op);      // Send the code (inverted because it's pindirs)
    pio_sm_get_blocking(mdio_pio, mdio_sm);           // Read the dummy value
    rc = pio_sm_get_blocking(mdio_pio, mdio_sm);
    printf("Read r=%d (op: %08x) (rb: %08x)\r\n", reg, op, rc);
    return (rc & 0xffff);
}


void mmio_write(int addr, int reg, uint16_t value) {
    uint32_t op = (1 << 30) | (1 << 28) | (addr << 23) | (reg << 18) | (0b10 << 16) | value;
    uint32_t rc;
    pio_sm_put_blocking(mdio_pio, mdio_sm, 0);        // Send all ones...
    pio_sm_put_blocking(mdio_pio, mdio_sm, ~op);      // Send the code (inverted because it's pindirs)
    pio_sm_get_blocking(mdio_pio, mdio_sm);           // Read the dummy value
    rc = pio_sm_get_blocking(mdio_pio, mdio_sm);
    printf("Write r=%d (op: %08x) (rb: %08x)\r\n", reg, op, rc);
}

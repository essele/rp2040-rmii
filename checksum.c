/**
 * @file checksum.c
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <string.h>
#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/interp.h"

//
// Useful macros from lwip, should be optmised by GCC
//
#define SWAP_BYTES_IN_WORD(w) (((w) & 0xff) << 8) | (((w) & 0xff00) >> 8)
#define FOLD_U32T(u)          ((uint32_t)(((u) >> 16) + ((u) & 0x0000ffffUL)))

/**
 * @brief Calculate an IP checkum using the RP2040 Interp module
 * 
 * This used to the RP2040 interp0 module to do the addition and can handle
 * two parallel 16bit additions concurrently, so it's slightly quicker than
 * the best of the standard LWIP offerings.
 * 
 * At present with a deliberately mixed workload it's between 28% and 30%
 * quicker. It's a real shame you can't use DMA with the interp engines as
 * that would be stunning (and free the CPU up to deal with the start and
 * end bytes!) -- but sadly not.
 *
 * NOTE: this needs interp0 to be inialised, the power on state doesn't
 *       work, not sure which bits aren't configured.
 * 
 * @param data 
 * @param length 
 * @return uint16_t 
 */
uint16_t interp_ip_checksum(uint8_t *data, int length) {
    // We will have 1, 2, or 3 bytes of unaligned data at the beginning
    // so we can use that to set initial accumulator values...
    //uint over = (uint32_t)data % 4;
    uint over = (uint32_t)data & 0x03;

    if (over == 3) {
            interp0->accum[0] = *data++ << 8;
            length -= 1;
    } else if( over == 2) {
            interp0->accum[0] = *(uint16_t *)data;
            data += 2;
            length -= 2;
    } else if(over == 1) {
            interp0->accum[0] = *data++ << 8;
            interp0->add_raw[0] = *(uint16_t *)data;
            data += 2;
            length -= 3;
    } else {
        interp0->accum[0] = 0;
    }
    interp0->accum[1] = 0;

    // Now we can cycle through all of the words (done this way so we check
    // for zero in the loop and not another constant)...
    uint32_t *ptr = (uint32_t *)data;
    uint words = length & 0xfffffffc;

    while(words) {
        interp0_hw->base01 = *ptr++;
        interp0_hw->pop[2];
        words -= 4;
    }

    // Now we can go back to an 8 but pointer and cover the ending 1, 2, or
    // 3 bytes in a simlar way to the start...
    data = (uint8_t *)ptr;
    if (length & 2) {
        interp0_hw->add_raw[0] = *(uint16_t *)data;
        data += 2;
    }
    if (length & 1) {
        interp0_hw->add_raw[0] = *data << 8;
    }    

    // base2 is a free add into result2 at this point

    // Now we can calculate the result so far...
    uint32_t res;
    res = interp0_hw->peek[2];
    res = FOLD_U32T(res);
    res = FOLD_U32T(res);

    // If it was odd, we swap the result...
//    uint32_t rc = (uint16_t)res;
//    if (over & 1) {
//        rc = SWAP_BYTES_IN_WORD(rc);
//    }

    uint16_t rc = (over & 1) ? SWAP_BYTES_IN_WORD(rc) : res;

    return (uint16_t)rc;
}

void interp_ip_checksum_init() {
    // Setup interp0...
    interp_config cfg = interp_default_config();
    interp_set_config(interp0, 0, &cfg);
    interp_set_config(interp0, 1, &cfg);
}

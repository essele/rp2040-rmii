/**
 * @file mac_rx.h
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MAC_RX_H
#define __MAC_RX_H

#include <stdint.h>

//#define RX_MAX_BYTES    1446              // for testing large packet handling
#define RX_MAX_BYTES    1600
#define RX_FRAME_COUNT  8


struct rx_frame {
    struct rx_frame     *next;
    int                 length;
    int                 checksum;
    uint8_t             data[RX_MAX_BYTES];
};

void mac_rx_init();
void mac_rx_up(int speed);
void mac_rx_down();

// These will be called by the engine that processes the packets...
void rx_add_to_free_list(struct rx_frame *frame);
struct rx_frame *rx_get_ready_frame();

void print_rx_stats();

#endif
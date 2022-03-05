/**
 * @file mac_tx.h
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */

void mac_tx_init(uint pin_tx0, uint pin_txen, uint pin_crs);
void mac_tx_teardown();
void mac_tx_send(uint8_t *data, uint length);
void mac_tx_test();

void mac_tx_up(int speed, int duplex);
void mac_tx_down();

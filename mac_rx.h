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

void mac_rx_init(uint pin_rx0, uint pin_crs);
void mac_rx_up(int speed);
void mac_rx_down();
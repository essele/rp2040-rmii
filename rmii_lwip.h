/**
 * @file rmii_lwip.h
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "lwip/netif.h"

struct netif *rmii_lwip_init(uint clkmhz, uint rx0, uint rx1, uint crs, uint tx0, uint tx1, uint txen, uint mdclk, uint mdio);
void rmii_lwip_poll(struct netif* netif);
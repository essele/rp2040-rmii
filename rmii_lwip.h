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
#include "lwip/def.h"
#include "lwip/netif.h"

struct netif *rmii_lwip_init();
void rmii_lwip_poll(struct netif* netif);
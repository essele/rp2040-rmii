/**
 * @file mdio.h
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-02-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#define LINK_NO_CHANGE              0
#define LINK_UP_10HD                1
#define LINK_UP_10FD                2
#define LINK_UP_100HD               3
#define LINK_UP_100FD               4
#define LINK_DOWN                   5
#define LINK_UP_UNKNOWN             99

int mdio_init(uint pin_mdc, uint pin_mdio);
int mdio_poll();
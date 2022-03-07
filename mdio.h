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

//#define MDIO_USE_IRQ 1

int mdio_init(uint pin_mdc, uint pin_mdio);
void mdio_poll();
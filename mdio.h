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

#define MMIO_BASIC_CONTROL          0
#define MMIO_BASIC_STATUS           1
#define MMIO_AUTONEG                4

int mdio_init(uint pin_mdc, uint pin_mdio);
uint16_t mmio_read(int addr, int reg);
void mmio_write(int addr, int reg, uint16_t value);
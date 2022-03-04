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

#define MDIO_BASIC_CONTROL          0
#define MDIO_BASIC_STATUS           1
#define MDIO_PHY_ID2                3
#define MDIO_AUTONEG_ADV            4
#define MDIO_SPECIAL_MODES          18
#define MDIO_SPECIAL_STATUS         31

int mdio_init(uint pin_mdc, uint pin_mdio);
uint16_t mdio_read(int addr, int reg);
void mdio_write(int addr, int reg, uint16_t value);
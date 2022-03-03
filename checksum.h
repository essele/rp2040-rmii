/**
 * @file checksum.h
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

uint16_t interp_ip_checksum(uint8_t *data, int length);
void interp_ip_checksum_init();
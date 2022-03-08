/**
 * @file pio_utils.h
 * @author Lee Essen (lee.essen@nowonline.co.uk)
 * @brief 
 * @version 0.1
 * @date 2022-03-08
 * 
 * @copyright Copyright (c) 2022
 *
 * Just a simple macro and struct to make it easier to choose between a list of
 * PIO programs 
 */

#include "hardware/pio.h"

#define PIO_PROG(name)          &name##_program, name##_program_get_default_config

struct pio_prog {
    const struct pio_program    *program;
    pio_sm_config               (*config_func)(uint offset);
};


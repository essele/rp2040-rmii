cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(rp2040_rmii_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(rp2040_rmii
    main.c
    mdio.c mdio.h
)

#pico_enable_stdio_usb(rp2040_rmii 1)
#pico_enable_stdio_uart(rp2040_rmii 0)   # why does this break it? (should it be 1?)
pico_add_extra_outputs(rp2040_rmii)

pico_generate_pio_header(rp2040_rmii ${CMAKE_CURRENT_LIST_DIR}/mdio.pio)
pico_generate_pio_header(rp2040_rmii ${CMAKE_CURRENT_LIST_DIR}/macrx.pio)

target_compile_definitions(rp2040_rmii PRIVATE
  PICO_DEFAULT_UART_TX_PIN=16
  PICO_DEFAULT_UART_RX_PIN=17
  PICO_DEFAULT_UART=0
)


target_link_libraries(rp2040_rmii 
  pico_stdlib 
  hardware_pio
  hardware_dma
  )

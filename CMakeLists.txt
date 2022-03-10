cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(rp2040_rmii_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(rp2040_rmii
    main.c
    rmii_lwip.c rmii_lwip.h
    mdio.c mdio.h mdio.pio.h mdio.pio
    mac_rx.c mac_rx.h mac_rx.pio.h mac_rx.pio
    mac_tx.c mac_tx.h mac_tx.pio.h mac_tx.pio
    checksum.c checksum.h
    clock50.pio
)

#pico_enable_stdio_usb(rp2040_rmii 1)
#pico_enable_stdio_uart(rp2040_rmii 0)   # why does this break it? (should it be 1?)
pico_add_extra_outputs(rp2040_rmii)

pico_generate_pio_header(rp2040_rmii ${CMAKE_CURRENT_LIST_DIR}/clock50.pio)
pico_generate_pio_header(rp2040_rmii ${CMAKE_CURRENT_LIST_DIR}/mdio.pio)
pico_generate_pio_header(rp2040_rmii ${CMAKE_CURRENT_LIST_DIR}/mac_rx.pio)
pico_generate_pio_header(rp2040_rmii ${CMAKE_CURRENT_LIST_DIR}/mac_tx.pio)

target_compile_definitions(rp2040_rmii PRIVATE
  PICO_DEFAULT_UART_TX_PIN=16
  PICO_DEFAULT_UART_RX_PIN=17
  PICO_DEFAULT_UART=0
  RMII_USE_LWIP=1
)

# Make sure we can find lwipopts.h...
target_include_directories(rp2040_rmii PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(rp2040_rmii 
  pico_stdlib 
  hardware_pio
  hardware_dma
  hardware_interp
  hardware_sync
  lwip
)

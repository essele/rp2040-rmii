cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(rmii_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_library(rmii INTERFACE)

target_sources(rmii INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/rmii_lwip.c 
    ${CMAKE_CURRENT_LIST_DIR}/rmii_lwip.h
    ${CMAKE_CURRENT_LIST_DIR}/mdio.c 
    ${CMAKE_CURRENT_LIST_DIR}/mdio.h 
    ${CMAKE_CURRENT_LIST_DIR}/mdio.pio
    ${CMAKE_CURRENT_LIST_DIR}/mac_rx.c 
    ${CMAKE_CURRENT_LIST_DIR}/mac_rx.h 
    ${CMAKE_CURRENT_LIST_DIR}/mac_rx.pio
    ${CMAKE_CURRENT_LIST_DIR}/mac_tx.c 
    ${CMAKE_CURRENT_LIST_DIR}/mac_tx.h 
    ${CMAKE_CURRENT_LIST_DIR}/mac_tx.pio
    ${CMAKE_CURRENT_LIST_DIR}/checksum.c 
    ${CMAKE_CURRENT_LIST_DIR}/checksum.h
    ${CMAKE_CURRENT_LIST_DIR}/debug.h
)

pico_generate_pio_header(rmii ${CMAKE_CURRENT_LIST_DIR}/mdio.pio)
pico_generate_pio_header(rmii ${CMAKE_CURRENT_LIST_DIR}/mac_rx.pio)
pico_generate_pio_header(rmii ${CMAKE_CURRENT_LIST_DIR}/mac_tx.pio)

target_compile_definitions(rmii INTERFACE
  RMII_USE_LWIP=1

  RMII_CLK_MHZ=150

  #
  # Main debugging on/off switch (if true), this turns all the others off
  # if disabled.
  #
  RMII_DEBUG=1

  #
  # Look at a few bytes of the packets coming in or going out, this will
  # have a significant impact on performance as it's generally printed
  # in an isr or time critical function.
  #
  RMII_DEBUG_PKT_RX=1
  RMII_DEBUG_PKT_TX=0

  #
  # Show LINK UP and DOWN changes
  #
  RMII_DEBUG_LINK=1

  #
  # MAC_ADDR:   overrides the complete MAC address
  # MAC_PREFIX: defined the first 3 octets, remaining based on uniqueid
  # DEFAULT:    use RPI prefix, remaining based on uniqieid
  #
  #RMII_USE_MAC_ADDR=0x45,0x46,0x47,0x48,0x49,0x4a
  #RMII_USE_MAC_PREFIX=0x44,0x44,0x44
)

# Make sure we can find lwipopts.h...
target_include_directories(rmii INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(rmii INTERFACE
  pico_stdlib 
  hardware_pio
  hardware_dma
  hardware_interp
  hardware_sync
  pico_unique_id
  lwip
)

#------------------------------------------------------------------------------
# Now see if we need to add the test target (only done if we are the top level
# project (i.e. not being included into something else.)
#------------------------------------------------------------------------------

get_directory_property(hasParent PARENT_DIRECTORY)
if(NOT hasParent)

add_executable(test
    main.c
)

target_compile_definitions(test PRIVATE
  PICO_DEFAULT_UART_TX_PIN=16
  PICO_DEFAULT_UART_RX_PIN=17
  PICO_DEFAULT_UART=0
)

#pico_enable_stdio_usb(test 1)
pico_enable_stdio_uart(test 1)
pico_add_extra_outputs(test)

target_link_libraries(test pico_stdlib rmii)

endif()

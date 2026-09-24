set(target_name host_bulk_cdc_stream)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 host_bulk_cdc_stream.c
 usb_descriptors.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/dcd_pio_usb.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )

# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

# Size of the ring the host frame handler writes the stream into. A power of
# two in 4096..65536. A bigger ring rides out longer stalls of the consumer.
set(HOST_BULK_RING_SIZE 32768 CACHE STRING "Bulk stream ring buffer size in bytes")

# pico_pio_usb is an INTERFACE library, so these definitions apply to its
# sources too.
target_compile_definitions(${target_name} PRIVATE
  # use tinyusb implementation
  PIO_USB_USE_TINYUSB
  PIO_USB_HOST_BULK_STREAM=1
  BULK_RING_SIZE=${HOST_BULK_RING_SIZE}
  )

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore pico_pio_usb tinyusb_device tinyusb_host)
pico_add_extra_outputs(${target_name})

# Companion device firmware that produces the stream. It only uses the
# native USB port, so it runs on any RP2040/RP2350 board.
add_subdirectory(sender)

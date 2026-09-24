set(target_name bulk_cdc_sender)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 bulk_cdc_sender.c
 usb_descriptors.c
 )

# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

# Limit the stream to this many bytes per second, 0 for as fast as the host
# takes it.
set(BULK_SENDER_BYTES_PER_SEC 0 CACHE STRING "Sender rate limit in bytes per second, 0 for none")
target_compile_definitions(${target_name} PRIVATE
  SENDER_BYTES_PER_SEC=${BULK_SENDER_BYTES_PER_SEC}
  )

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib tinyusb_device)
pico_add_extra_outputs(${target_name})

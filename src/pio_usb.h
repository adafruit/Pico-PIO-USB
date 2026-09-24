
#pragma once

#include "pio_usb_configuration.h"
#include "usb_definitions.h"

#if PIO_USB_HOST_BULK_STREAM
#include "pio_usb_bulk_stream.h"
#endif

#ifdef __cplusplus
 extern "C" {
#endif

// Host functions
usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c);
int pio_usb_host_add_port(uint8_t pin_dp, PIO_USB_PINOUT pinout);
void pio_usb_host_task(void);
void pio_usb_host_stop(void);
void pio_usb_host_restart(void);
uint32_t pio_usb_host_get_frame_number(void);

// Call this every 1ms when skip_alarm_pool is true.
void pio_usb_host_frame(void);

#if PIO_USB_HOST_ISOCHRONOUS
// Supply storage for the isochronous IN ring. size must be a power of two,
// >= 2048. Call before any isochronous transfer is queued (buffer == NULL
// with size 0 disables the ring again). Returns false on a bad size or if
// the ring is currently attached to an endpoint. Without a ring, isochronous
// IN transfers still work in the per-transfer (one packet per frame) mode.
bool pio_usb_host_set_iso_ring(uint8_t *buffer, uint32_t size);
#endif

// Device functions
usb_device_t *pio_usb_device_init(const pio_usb_configuration_t *c,
                                  const usb_descriptor_buffers_t *buffers);
void pio_usb_device_task(void);

// Common functions
endpoint_t *pio_usb_get_endpoint(usb_device_t *device, uint8_t idx);
int pio_usb_get_in_data(endpoint_t *ep, uint8_t *buffer, uint8_t len);
int pio_usb_set_out_data(endpoint_t *ep, const uint8_t *buffer, uint8_t len);

#ifdef __cplusplus
 }
#endif

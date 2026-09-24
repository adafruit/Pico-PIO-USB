/**
 * Copyright (c) 2026 pt (Adafruit Industries)
 *
 *
 * Host bulk IN streaming (PIO_USB_HOST_BULK_STREAM).
 *
 * A full-speed bulk IN endpoint is polled by the host frame handler every
 * frame, with extra packets in otherwise idle frame time, and the payload is
 * written into a caller-owned ring buffer. The application drains the ring
 * from another core without any host controller call in between, so the
 * device keeps streaming while the application is busy. This is what devices
 * such as RTL2832U SDR dongles need: they stream continuously over a bulk
 * endpoint at rates well above one packet per frame.
 *
 * The ring is single-producer (host frame core), single-consumer
 * (application) and lock-free. The ring descriptor and its storage must be
 * in internal SRAM; both stay owned by the application, which must not free
 * them until stop() has returned true (or stats report active == false).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Counters kept by the host frame core. All fields are uint32_t in this
// exact order, so the block can be copied out as an array of 18 words.
typedef struct {
  uint32_t polls;                // IN tokens sent
  uint32_t packets;              // data packets accepted into the ring
  uint32_t bytes;                // payload bytes accepted into the ring
  uint32_t zero_packets;         // accepted packets with no payload
  uint32_t error_packets;        // timeouts, CRC/PID errors, STALL
  uint32_t overrun_packets;      // packets dropped because the ring was full
  uint32_t overrun_bytes;        // payload bytes of those packets
  uint32_t unarmed_frames;       // frames in which the port could not be polled
  uint32_t active_frames;        // frames in which the ring was attached
  uint32_t total_transaction_us; // time spent on the bus for this endpoint
  uint32_t max_transaction_us;   // longest single transaction
  uint32_t max_poll_gap_us;      // longest gap between two polls
  uint32_t high_water_bytes;     // most bytes ever waiting in the ring
  uint32_t max_frame_us;         // longest host frame while attached
  uint32_t frame_overruns;       // host frames that took longer than 1 ms
  uint32_t available_bytes;      // filled by stats(): bytes waiting to be read
  uint32_t active;               // filled by stats(): ring is attached
  uint32_t disconnected;         // the device went away while attached
} pio_usb_bulk_stats_t;

typedef struct pio_usb_bulk_ring {
  uint8_t *buffer;
  uint32_t capacity;             // power of two, 4096..65536
  volatile uint32_t write_pos;   // free-running; producer (frame core) only
  volatile uint32_t read_pos;    // free-running; consumer only
  volatile bool stop_requested;  // consumer -> producer
  volatile bool active;          // producer clears as its final access
  uint32_t last_poll_us;
  volatile uint32_t stats_seq;   // seqlock, odd while stats are being written
  pio_usb_bulk_stats_t stats;
} pio_usb_bulk_ring_t;

// Attach a ring to an open full-speed bulk IN endpoint (packet size <= 64)
// and start polling it. capacity is the size of storage, a power of two in
// 4096..65536. Fails if the endpoint is unknown, of the wrong type, behind a
// low-speed hub, has a transfer queued, or already streams.
bool pio_usb_host_bulk_stream_start(uint8_t root_idx, uint8_t device_address,
                                    uint8_t ep_address, pio_usb_bulk_ring_t *ring,
                                    uint8_t *storage, uint32_t capacity);

// Copy up to len bytes out of the ring. Returns the number copied, which is
// 0 when nothing is waiting. Call from one consumer context only.
uint32_t pio_usb_host_bulk_stream_read(pio_usb_bulk_ring_t *ring, uint8_t *dest,
                                       uint32_t len);

// Ask the frame core to detach the ring and wait up to timeout_us for it to
// do so. Returns false on timeout: the ring is still in use and must not be
// freed; call again later. Must not be called from the frame handler itself.
bool pio_usb_host_bulk_stream_stop(pio_usb_bulk_ring_t *ring, uint32_t timeout_us);

// Take a consistent snapshot of the counters. Returns false if the producer
// kept the stats locked for more than 2 ms, which means it is not running.
bool pio_usb_host_bulk_stream_stats(pio_usb_bulk_ring_t *ring,
                                    pio_usb_bulk_stats_t *out);

#ifdef __cplusplus
}
#endif

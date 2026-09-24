/**
 * Copyright (c) 2021 sekigon-gonnoc
 *                    Ha Thach (thach@tinyusb.org)
 */

#pragma GCC push_options
#pragma GCC optimize("-O3")

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/sync.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "pio_usb.h"
#include "pio_usb_ll.h"
#include "usb_crc.h"

enum {
  TRANSACTION_MAX_RETRY = 3, // Number of times to retry a failed transaction
};

static alarm_pool_t *_alarm_pool = NULL;
static repeating_timer_t sof_rt;
// The sof_count may be incremented and then read on different cores.
static volatile uint32_t sof_count = 0;
static bool timer_active;

static volatile bool cancel_timer_flag;
static volatile bool start_timer_flag;
static __unused uint32_t int_stat;
static uint8_t sof_packet[4] = {USB_SYNC, USB_PID_SOF, 0x00, 0x10};
static uint8_t sof_packet_encoded[4 * 2 * 7 / 6 + 2];
static uint8_t sof_packet_encoded_len;
static uint8_t keepalive_encoded[1];

static bool sof_timer(repeating_timer_t *_rt);

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

static void start_timer(alarm_pool_t *alarm_pool) {
  if (timer_active) {
    return;
  }

  if (alarm_pool != NULL) {
    alarm_pool_add_repeating_timer_us(alarm_pool, -1000, sof_timer, NULL,
                                      &sof_rt);
  }

  timer_active = true;
}

static __unused void stop_timer(void) {
  cancel_repeating_timer(&sof_rt);
  timer_active = false;
}

usb_device_t *pio_usb_host_init(const pio_usb_configuration_t *c) {
  pio_port_t *pp = PIO_USB_PIO_PORT(0);
  root_port_t *root = PIO_USB_ROOT_PORT(0);

  pio_usb_bus_init(pp, c, root);
  root->mode = PIO_USB_MODE_HOST;

  float const cpu_freq = (float)clock_get_hz(clk_sys);
  pio_calculate_clkdiv_from_float(cpu_freq / 48000000,
                                  &pp->clk_div_fs_tx.div_int,
                                  &pp->clk_div_fs_tx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 6000000,
                                  &pp->clk_div_ls_tx.div_int,
                                  &pp->clk_div_ls_tx.div_frac);

  pio_calculate_clkdiv_from_float(cpu_freq / 96000000,
                                  &pp->clk_div_fs_rx.div_int,
                                  &pp->clk_div_fs_rx.div_frac);
  pio_calculate_clkdiv_from_float(cpu_freq / 12000000,
                                  &pp->clk_div_ls_rx.div_int,
                                  &pp->clk_div_ls_rx.div_frac);

  sof_packet_encoded_len =
      pio_usb_ll_encode_tx_data(sof_packet, sizeof(sof_packet), sof_packet_encoded);
  pio_usb_ll_encode_tx_data(NULL, 0, keepalive_encoded);

  if (!c->skip_alarm_pool) {
    _alarm_pool = c->alarm_pool;
    if (!_alarm_pool) {
      _alarm_pool = alarm_pool_create(2, 1);
    }
  }
  start_timer(_alarm_pool);

  return &pio_usb_device[0];
}

void pio_usb_host_stop(void) {
  cancel_timer_flag = true;
  while (cancel_timer_flag) {
    continue;
  }
}

void pio_usb_host_restart(void) {
  start_timer_flag = true;
  while (start_timer_flag) {
    continue;
  }
}

//--------------------------------------------------------------------+
// Bus functions
//--------------------------------------------------------------------+

static void __no_inline_not_in_flash_func(override_pio_program)(PIO pio, const pio_program_t* program, uint offset) {
    for (uint i = 0; i < program->length; ++i) {
      uint16_t instr = program->instructions[i];
      pio->instr_mem[offset + i] =
          pio_instr_bits_jmp != _pio_major_instr_bits(instr) ? instr
                                                             : instr + offset;
    }
}

static __always_inline void override_pio_rx_program(PIO pio,
                                             const pio_program_t *program,
                                             const pio_program_t *debug_program,
                                             uint offset, int debug_pin) {
  if (debug_pin < 0) {
    override_pio_program(pio, program, offset);
  } else {
    override_pio_program(pio, debug_program, offset);
  }
}

static void
__no_inline_not_in_flash_func(configure_tx_program)(pio_port_t *pp,
                                                    root_port_t *port) {
  if (port->pinout == PIO_USB_PINOUT_DPDM) {
    pp->fs_tx_program = &usb_tx_dpdm_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dpdm_program;
    pp->ls_tx_program = &usb_tx_dmdp_program;
  } else {
    pp->fs_tx_program = &usb_tx_dmdp_program;
    pp->fs_tx_pre_program = &usb_tx_pre_dmdp_program;
    pp->ls_tx_program = &usb_tx_dpdm_program;
  }
}

static void __no_inline_not_in_flash_func(configure_fullspeed_host)(
    pio_port_t *pp, root_port_t *port) {
  pp->low_speed = false;
  configure_tx_program(pp, port);
  pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  override_pio_program(pp->pio_usb_tx, pp->fs_tx_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);
  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);
  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_reset_instr);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dp);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);
}

static void __no_inline_not_in_flash_func(configure_lowspeed_host)(
    pio_port_t *pp, root_port_t *port) {
  pp->low_speed = true;
  configure_tx_program(pp, port);
  pio_sm_clear_fifos(pp->pio_usb_tx, pp->sm_tx);
  override_pio_program(pp->pio_usb_tx, pp->ls_tx_program, pp->offset_tx);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_ls_tx);
  usb_tx_configure_pins(pp->pio_usb_tx, pp->sm_tx, port->pin_dp, port->pin_dm);
  pio_sm_exec(pp->pio_usb_tx, pp->sm_tx, pp->tx_reset_instr);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_rx, port->pin_dm);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);

  pio_sm_set_jmp_pin(pp->pio_usb_rx, pp->sm_eop, port->pin_dp);
  pio_sm_set_in_pins(pp->pio_usb_rx, pp->sm_eop, port->pin_dm);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_ls_rx);
}

static void __no_inline_not_in_flash_func(configure_root_port)(
    pio_port_t *pp, root_port_t *root) {
  if (root->is_fullspeed) {
    configure_fullspeed_host(pp, root);
  } else {
    configure_lowspeed_host(pp, root);
  }
}

static void __no_inline_not_in_flash_func(restore_fs_bus)(pio_port_t *pp) {
  // change bus speed to full-speed
  pp->low_speed = false;
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, false);
  SM_SET_CLKDIV(pp->pio_usb_tx, pp->sm_tx, pp->clk_div_fs_tx);
  pio_sm_set_enabled(pp->pio_usb_tx, pp->sm_tx, true);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  SM_SET_CLKDIV_MAXSPEED(pp->pio_usb_rx, pp->sm_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, true);

  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, false);
  SM_SET_CLKDIV(pp->pio_usb_rx, pp->sm_eop, pp->clk_div_fs_rx);
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_eop, true);
}

// Time about 1us ourselves so it lives in RAM.
static void __not_in_flash_func(busy_wait_1_us)(void) {
  uint32_t start = get_time_us_32();
  while (get_time_us_32() == start) {
      tight_loop_contents();
  }
}

static void end_transaction(pio_port_t *pp);
#if PIO_USB_HOST_BULK_STREAM
static void bulk_stream_detach(endpoint_t *ep, bool disconnected);
#endif

static bool __no_inline_not_in_flash_func(connection_check)(root_port_t *port) {
  if (pio_usb_bus_get_line_state(port) == PORT_PIN_SE0) {
    busy_wait_1_us();

    if (pio_usb_bus_get_line_state(port) == PORT_PIN_SE0) {
      busy_wait_1_us();
      // device disconnect
      port->connected = false;
      port->suspended = true;
      port->ints |= PIO_USB_INTS_DISCONNECT_BITS;

      // failed/retired all queuing transfer in this root
      uint8_t root_idx = port - PIO_USB_ROOT_PORT(0);
      for (int ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
        endpoint_t *ep = PIO_USB_ENDPOINT(ep_idx);
#if PIO_USB_HOST_BULK_STREAM
        if ((ep->root_idx == root_idx) && ep->bulk_ring) {
          bulk_stream_detach(ep, true);
        }
#endif
        if ((ep->root_idx == root_idx) && ep->size && ep->has_transfer) {
          pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
        }
      }

      return false;
    }
  }

  return true;
}

//--------------------------------------------------------------------+
// SOF
//--------------------------------------------------------------------+
static int usb_setup_transaction(pio_port_t *pp, endpoint_t *ep);
static int usb_in_transaction(pio_port_t *pp, endpoint_t *ep);
static int usb_out_transaction(pio_port_t *pp, endpoint_t *ep);

#if PIO_USB_HOST_ISOCHRONOUS
static endpoint_t *iso_ring_ep;
static uint8_t *iso_ring;      // runtime storage; NULL = ring disabled
static uint32_t iso_ring_mask; // size - 1; size is a power of two
static uint32_t iso_ring_head;
static uint32_t iso_ring_count;
static bool iso_ring_lost;
static uint8_t iso_ring_silent; // consecutive frames without a response

// The ring helpers run on the frame core and must stay out of flash, so the
// ring size is required to be a power of two: a runtime `%` would emit a
// __aeabi_uidivmod libcall in flash on Cortex-M0+.
static void __no_inline_not_in_flash_func(iso_ring_copy_in)(uint8_t const *src, uint32_t len) {
  uint32_t const tail = (iso_ring_head + iso_ring_count) & iso_ring_mask;
  uint32_t first = (iso_ring_mask + 1) - tail;
  if (first > len) {
    first = len;
  }
  memcpy(&iso_ring[tail], src, first);
  memcpy(iso_ring, src + first, len - first);
  iso_ring_count += len;
}

static void __no_inline_not_in_flash_func(iso_ring_copy_out)(uint8_t *dst, uint32_t len) {
  uint32_t first = (iso_ring_mask + 1) - iso_ring_head;
  if (first > len) {
    first = len;
  }
  if (dst) {
    memcpy(dst, &iso_ring[iso_ring_head], first);
    memcpy(dst + first, iso_ring, len - first);
  }
  iso_ring_head = (iso_ring_head + len) & iso_ring_mask;
  iso_ring_count -= len;
}

// A record longer than what the ring holds means the ring is corrupted.
// Empty it and report a gap.
static bool __no_inline_not_in_flash_func(iso_ring_check_len)(uint32_t len) {
  if (len <= iso_ring_count) {
    return true;
  }
  iso_ring_head = 0;
  iso_ring_count = 0;
  iso_ring_lost = true;
  return false;
}

// Length of the record at the head of the ring, header included. A lost
// marker (0xffff) is a record of just its two header bytes.
static uint32_t __no_inline_not_in_flash_func(iso_ring_head_len)(void) {
  uint16_t const value = iso_ring[iso_ring_head] |
                         (iso_ring[(iso_ring_head + 1) & iso_ring_mask] << 8);
  return value == 0xffff ? 2 : value + 2u;
}

// Make room for a new record by dropping the oldest one, so the ring always
// holds the newest part of the stream when the application falls behind.
// One lost marker stays at the head to show where the gap is: it is put in
// front of the oldest surviving record, or kept there if already present.
static void __no_inline_not_in_flash_func(iso_ring_drop_oldest)(void) {
  if (iso_ring_count < 2 || !iso_ring_check_len(iso_ring_head_len())) {
    iso_ring_count = 0;
    iso_ring_lost = true;
    return;
  }
  if (iso_ring_head_len() == 2) {
    iso_ring_copy_out(NULL, 2); // the marker; put back below
    if (iso_ring_count < 2 || !iso_ring_check_len(iso_ring_head_len())) {
      return;
    }
  }
  iso_ring_copy_out(NULL, iso_ring_head_len());
  iso_ring_head = (iso_ring_head - 2) & iso_ring_mask; // unsigned wrap is fine
  iso_ring_count += 2;
  iso_ring[iso_ring_head] = 0xff;
  iso_ring[(iso_ring_head + 1) & iso_ring_mask] = 0xff;
}

static void __no_inline_not_in_flash_func(iso_ring_receive)(endpoint_t *ep, int len,
                                                           uint8_t pid, uint8_t const *data) {
  uint8_t const lost_marker[2] = {0xff, 0xff};
  if (len < 0) {
    if (pid == USB_PID_DATA0 || pid == USB_PID_DATA1) {
      iso_ring_lost = true; // packet arrived corrupted
    } else if (++iso_ring_silent >= 32) {
      // The device stopped streaming (alternate setting 0). Stop polling
      // until the application queues another transfer. A transfer pending
      // now fails on the next frame.
      iso_ring_ep = NULL;
    }
    len = 0;
  } else {
    iso_ring_silent = 0;
  }
  if (iso_ring_lost) {
    while (iso_ring_count + 2 > iso_ring_mask + 1) {
      iso_ring_drop_oldest();
    }
    iso_ring_copy_in(lost_marker, 2);
    iso_ring_lost = false;
  }
  if (len > 0) {
    while (iso_ring_count + 2 + len > iso_ring_mask + 1) {
      iso_ring_drop_oldest();
    }
    uint8_t const header[2] = {len & 0xff, len >> 8};
    iso_ring_copy_in(header, 2);
    iso_ring_copy_in(data, len);
  }

  if (!ep->transfer_started || !ep->has_transfer || ep->transfer_aborted) {
    return;
  }
  // Hand over as many whole records as fit in the transfer buffer.
  while (iso_ring_count >= 2) {
    uint32_t const rec_len = iso_ring_head_len();
    if (!iso_ring_check_len(rec_len)) {
      break;
    }
    if (rec_len > ep->total_len) {
      iso_ring_copy_out(NULL, rec_len); // can never fit; drop it
      iso_ring_lost = true;
      continue;
    }
    if (ep->actual_len + rec_len > ep->total_len) {
      break;
    }
    iso_ring_copy_out(ep->app_buf + ep->actual_len, rec_len);
    ep->actual_len += rec_len;
  }
  if (ep->actual_len > 0) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
  }
}

bool pio_usb_host_set_iso_ring(uint8_t *buffer, uint32_t size) {
  if (iso_ring_ep != NULL) {
    return false; // in use by an endpoint; stop streaming first
  }
  if (buffer == NULL) {
    if (size != 0) {
      return false;
    }
    iso_ring = NULL;
    iso_ring_mask = 0;
    return true;
  }
  if (size < 2048 || (size & (size - 1))) {
    return false;
  }
  iso_ring = buffer;
  iso_ring_mask = size - 1;
  iso_ring_head = 0;
  iso_ring_count = 0;
  iso_ring_lost = false;
  return true;
}
#endif

#if PIO_USB_HOST_BULK_STREAM
//--------------------------------------------------------------------+
// Bulk stream ring, producer side. Everything here runs on the host frame
// core only. The ring and its storage belong to the application; the
// application core only ever reads write_pos and writes read_pos.
//--------------------------------------------------------------------+
_Static_assert(sizeof(pio_usb_bulk_stats_t) == 18 * sizeof(uint32_t),
               "pio_usb_bulk_stats_t layout is part of the API");

enum {
  BULK_STREAM_BURST_MAX_PACKETS = 15,  // extra transactions after the normal pass
  BULK_STREAM_BURST_DEADLINE_US = 800, // no new transaction after this frame time
  BULK_STREAM_MIN_CAPACITY = 4096,
  BULK_STREAM_MAX_CAPACITY = 65536,
};

// Seqlock around the stats: odd sequence means an update is in progress and
// the consumer's snapshot must be retried.
static __always_inline void bulk_stats_begin(pio_usb_bulk_ring_t *ring) {
  ring->stats_seq++;
  __dmb();
}

static __always_inline void bulk_stats_end(pio_usb_bulk_ring_t *ring) {
  __dmb();
  ring->stats_seq++;
}

// Release the endpoint. The application may free the ring as soon as it
// sees active == false, so that store must be the producer's last access.
static void __no_inline_not_in_flash_func(bulk_stream_detach)(endpoint_t *ep,
                                                             bool disconnected) {
  pio_usb_bulk_ring_t *ring = ep->bulk_ring;
  if (!ring) {
    return;
  }
  bulk_stats_begin(ring);
  ring->stats.disconnected |= disconnected;
  bulk_stats_end(ring);
  ep->bulk_ring = NULL;
  __dmb();
  ring->active = false;
}

// Append one packet. When the ring is full the packet is dropped and
// counted: the stream is continuous up to the point of loss, and the
// application can see from overrun_* that it fell behind.
static void __no_inline_not_in_flash_func(bulk_ring_write)(pio_usb_bulk_ring_t *ring,
                                                          const uint8_t *data,
                                                          uint16_t len) {
  uint32_t const written = ring->write_pos;
  uint32_t const read = ring->read_pos;
  __dmb();
  uint32_t const used = written - read;
  if (used > ring->capacity || len > ring->capacity - used) {
    ring->stats.overrun_packets++;
    ring->stats.overrun_bytes += len;
    return;
  }
  uint32_t const offset = written & (ring->capacity - 1);
  uint32_t first = ring->capacity - offset;
  if (first > len) {
    first = len;
  }
  if (first) {
    memcpy(ring->buffer + offset, data, first);
  }
  if (len > first) {
    memcpy(ring->buffer, data + first, len - first);
  }
  if (used + len > ring->stats.high_water_bytes) {
    ring->stats.high_water_bytes = used + len;
  }
  __dmb(); // publish only after the whole packet is in place
  ring->write_pos = written + len;
}

// One bulk IN transaction into the ring, with ACK and DATA0/1 toggle.
// Returns 1 for a new packet, 0 for NAK or a duplicate (the device missed
// our ACK: the repeat is ACKed and discarded) and -1 for an error. Bursting
// stops on anything but 1. No TinyUSB event is raised.
static int __no_inline_not_in_flash_func(usb_bulk_ring_transaction)(
    pio_port_t *pp, endpoint_t *ep, pio_usb_bulk_ring_t *ring) {
  uint32_t const started = get_time_us_32();
  uint8_t const expected = ep->data_id ? USB_PID_DATA1 : USB_PID_DATA0;

  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_send_token(pp, USB_PID_IN, ep->dev_addr, ep->ep_num);
  pio_usb_bus_start_receive(pp);
  int const len = pio_usb_bus_receive_packet_and_handshake_limit(pp, USB_PID_ACK, ep->size);
  uint8_t const pid = pp->usb_rx_buffer[1];
  end_transaction(pp);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  bulk_stats_begin(ring);
  if (ring->stats.polls) {
    uint32_t const gap = started - ring->last_poll_us;
    if (gap > ring->stats.max_poll_gap_us) {
      ring->stats.max_poll_gap_us = gap;
    }
  }
  ring->last_poll_us = started;
  ring->stats.polls++;

  int result = 0;
  if (len >= 0 && pid == expected) {
    ep->data_id ^= 1;
    ring->stats.packets++;
    ring->stats.bytes += len;
    ring->stats.zero_packets += len == 0;
    bulk_ring_write(ring, &pp->usb_rx_buffer[2], len);
    result = 1;
  } else if (len < 0 && pid == USB_PID_NAK) {
    // nothing to send yet; try again next frame
  } else if (len >= 0 && (pid == USB_PID_DATA0 || pid == USB_PID_DATA1)) {
    // duplicate of the previous packet, already stored
  } else {
    ring->stats.error_packets++;
    result = -1;
    if (pid == USB_PID_STALL) {
      ring->stop_requested = true;
    }
  }

  uint32_t const elapsed = get_time_us_32() - started;
  ring->stats.total_transaction_us += elapsed;
  if (elapsed > ring->stats.max_transaction_us) {
    ring->stats.max_transaction_us = elapsed;
  }
  bulk_stats_end(ring);
  return result;
}

// Runs first thing in every frame, before the timer and port readiness
// tests, so a stop request is honoured even while the port is suspended,
// disconnected, or the timer is being stopped.
static void __no_inline_not_in_flash_func(bulk_stream_frame_begin)(void) {
  for (int ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_idx);
    pio_usb_bulk_ring_t *ring = ep->bulk_ring;
    __dmb();
    if (!ring) {
      continue;
    }
    root_port_t *root = PIO_USB_ROOT_PORT(ep->root_idx);
    if (ring->stop_requested || !ep->size || !root->connected) {
      bulk_stream_detach(ep, !root->connected);
      continue;
    }
    bulk_stats_begin(ring);
    ring->stats.active_frames++;
    if (!timer_active || !root->initialized || root->suspended || ep->transfer_aborted) {
      ring->stats.unarmed_frames++;
    }
    bulk_stats_end(ring);
  }
}

static void __no_inline_not_in_flash_func(bulk_stream_frame_end)(uint32_t elapsed) {
  for (int ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
    pio_usb_bulk_ring_t *ring = PIO_USB_ENDPOINT(ep_idx)->bulk_ring;
    if (ring) {
      bulk_stats_begin(ring);
      if (elapsed > ring->stats.max_frame_us) {
        ring->stats.max_frame_us = elapsed;
      }
      ring->stats.frame_overruns += elapsed > 1000;
      bulk_stats_end(ring);
    }
  }
}

// Extra transactions in the idle part of the frame, after every endpoint
// has had its normal turn. Stops on NAK, duplicate or error, and starts no
// transaction after the deadline so the frame still ends within 1 ms.
static void __no_inline_not_in_flash_func(usb_bulk_stream_burst)(
    pio_port_t *pp, endpoint_t *ep, pio_usb_bulk_ring_t *ring, uint32_t frame_started) {
  for (unsigned extra = 0; extra < BULK_STREAM_BURST_MAX_PACKETS; extra++) {
    if (ring->stop_requested || ep->transfer_aborted ||
        (uint32_t)(get_time_us_32() - frame_started) >= BULK_STREAM_BURST_DEADLINE_US) {
      break;
    }
    if (usb_bulk_ring_transaction(pp, ep, ring) <= 0) {
      break;
    }
  }
}
#endif // PIO_USB_HOST_BULK_STREAM

#if PIO_USB_HOST_BULK_IN_BURST
// Same idea for an ordinary queued bulk IN transfer: drain as much of it as
// fits in this frame instead of one packet per frame. Stops as soon as a
// transaction makes no progress (NAK, duplicate, error) or the transfer
// completes.
static void __no_inline_not_in_flash_func(usb_bulk_in_burst)(pio_port_t *pp, endpoint_t *ep,
                                                            uint32_t frame_started) {
  if ((ep->attr & 0x03) != EP_ATTR_BULK || !(ep->ep_num & EP_IN) || ep->need_pre ||
      !ep->size || ep->size > PIO_USB_EP_SIZE) {
    return;
  }
  ep->transfer_started = true;
  for (unsigned extra = 0; extra < BULK_STREAM_BURST_MAX_PACKETS; extra++) {
    if (!ep->has_transfer || ep->transfer_aborted ||
        (uint32_t)(get_time_us_32() - frame_started) >= BULK_STREAM_BURST_DEADLINE_US) {
      break;
    }
    uint16_t const before = ep->actual_len;
    if (usb_in_transaction(pp, ep) < 0 || !ep->has_transfer || ep->actual_len == before) {
      break;
    }
  }
  ep->transfer_started = false;
}
#endif // PIO_USB_HOST_BULK_IN_BURST

void __not_in_flash_func(pio_usb_host_frame)(void) {
#if PIO_USB_HOST_BULK_STREAM || PIO_USB_HOST_BULK_IN_BURST
  uint32_t const frame_started = get_time_us_32();
#endif
#if PIO_USB_HOST_BULK_STREAM
  bulk_stream_frame_begin();
#endif
  if (!timer_active) {
    return;
  }

  pio_port_t *pp = PIO_USB_PIO_PORT(0);

  // Send SOF
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended &&
          connection_check(root))) {
      continue;
    }
    configure_root_port(pp, root);
    if (root->is_fullspeed) {
      // Send SOF for full speed
      pio_usb_bus_usb_transfer(pp, sof_packet_encoded, sof_packet_encoded_len);
    } else {
      // Send Keep alive for low speed
      pio_usb_bus_usb_transfer(pp, keepalive_encoded, 1);
    }
  }

  // Carry out all queued endpoint transaction
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended)) {
      continue;
    }

    configure_root_port(pp, root);

    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT;
         ep_pool_idx++) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
      if ((ep->root_idx == root_idx) && ep->size) {
        bool const is_periodic = ((ep->attr & 0x03) == EP_ATTR_INTERRUPT);

        if (is_periodic && (ep->interval_counter > 0)) {
          ep->interval_counter--;
          continue;
        }

#if PIO_USB_HOST_ISOCHRONOUS
        if (iso_ring != NULL &&
            (ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS && (ep->ep_num & EP_IN) &&
            ep != iso_ring_ep) {
          if (ep->has_transfer && !ep->transfer_aborted) {
            pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
          }
          continue;
        }
#endif
        // A ring-owned endpoint has no TinyUSB transfer, so it is polled
        // but transfer_started stays false: abort must not wait on it.
        bool const live = ep->has_transfer && !ep->transfer_aborted;
        bool active = live;
#if PIO_USB_HOST_ISOCHRONOUS
        active = active || ep == iso_ring_ep;
#endif
#if PIO_USB_HOST_BULK_STREAM
        pio_usb_bulk_ring_t *const bulk_ring = ep->bulk_ring;
        active = active || bulk_ring != NULL;
#endif
        if (active) {
          ep->transfer_started = live;

          if (ep->need_pre) {
            pp->need_pre = true;
          }

          if (ep->ep_num == 0 && ep->data_id == USB_PID_SETUP) {
            usb_setup_transaction(pp, ep);
          } else {
            if (ep->ep_num & EP_IN) {
#if PIO_USB_HOST_BULK_STREAM
              if (bulk_ring) {
                usb_bulk_ring_transaction(pp, ep, bulk_ring);
              } else {
                usb_in_transaction(pp, ep);
              }
#else
              usb_in_transaction(pp, ep);
#endif
            } else {
              usb_out_transaction(pp, ep);
            }

            if (is_periodic) {
              ep->interval_counter = ep->interval - 1;
            }
          }

          if (ep->need_pre) {
            pp->need_pre = false;
            restore_fs_bus(pp);
          }

          ep->transfer_started = false;
        }
      }
    }
  }

#if PIO_USB_HOST_BULK_STREAM || PIO_USB_HOST_BULK_IN_BURST
  // Extra bulk IN packets in the otherwise idle rest of the frame. Done
  // after the normal pass so control, interrupt and isochronous traffic keep
  // their slots. Full speed only: a low-speed frame has no room to spare.
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (!(root->initialized && root->connected && !root->suspended &&
          root->is_fullspeed)) {
      continue;
    }
    bool configured = false;
    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
      if (ep->root_idx != root_idx || !ep->size) {
        continue;
      }
#if PIO_USB_HOST_BULK_STREAM
      pio_usb_bulk_ring_t *const bulk_ring = ep->bulk_ring;
      if (bulk_ring) {
        if (!configured) {
          configure_root_port(pp, root);
          configured = true;
        }
        usb_bulk_stream_burst(pp, ep, bulk_ring, frame_started);
        continue;
      }
#endif
#if PIO_USB_HOST_BULK_IN_BURST
      if (ep->has_transfer && !ep->transfer_aborted) {
        if (!configured) {
          configure_root_port(pp, root);
          configured = true;
        }
        usb_bulk_in_burst(pp, ep, frame_started);
      }
#endif
    }
  }
#endif

  // check for new connection to root hub
  for (int root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
    if (root->initialized && !root->connected) {
      port_pin_status_t const line_state = pio_usb_bus_get_line_state(root);
      if (line_state == PORT_PIN_FS_IDLE || line_state == PORT_PIN_LS_IDLE) {
        root->is_fullspeed = (line_state == PORT_PIN_FS_IDLE);
        root->connected = true;
        root->suspended = true; // need a bus reset before operating
        root->ints |= PIO_USB_INTS_CONNECT_BITS;
      }
    }
  }

  // Invoke IRQHandler if interrupt status is set
  for (uint8_t root_idx = 0; root_idx < PIO_USB_ROOT_PORT_CNT; root_idx++) {
    if (PIO_USB_ROOT_PORT(root_idx)->ints) {
      pio_usb_host_irq_handler(root_idx);
    }
  }

#if PIO_USB_HOST_BULK_STREAM
  bulk_stream_frame_end(get_time_us_32() - frame_started);
#endif
  sof_count++;

  // SOF counter is 11-bit
  uint16_t const sof_count_11b = sof_count & 0x7ff;
  sof_packet[2] = sof_count_11b & 0xff;
  sof_packet[3] = (calc_usb_crc5(sof_count_11b) << 3) | (sof_count_11b >> 8);
  sof_packet_encoded_len =
      pio_usb_ll_encode_tx_data(sof_packet, sizeof(sof_packet), sof_packet_encoded);
}

static bool __no_inline_not_in_flash_func(sof_timer)(repeating_timer_t *_rt) {
  (void)_rt;

  pio_usb_host_frame();

  return true;
}

//--------------------------------------------------------------------+
// Host Controller functions
//--------------------------------------------------------------------+

uint32_t pio_usb_host_get_frame_number(void) {
  return sof_count;
}

void pio_usb_host_port_reset_start(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);

  // bus is not operating while in reset
  root->suspended = true;

  // Force line state to SE0
  gpio_set_outover(root->pin_dp,  GPIO_OVERRIDE_LOW);
  gpio_set_outover(root->pin_dm,  GPIO_OVERRIDE_LOW);
  gpio_set_oeover(root->pin_dp,  GPIO_OVERRIDE_HIGH);
  gpio_set_oeover(root->pin_dm,  GPIO_OVERRIDE_HIGH);
}

void pio_usb_host_port_reset_end(uint8_t root_idx) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);

  // line state to input
  gpio_set_oeover(root->pin_dp,  GPIO_OVERRIDE_NORMAL);
  gpio_set_oeover(root->pin_dm,  GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(root->pin_dp,  GPIO_OVERRIDE_NORMAL);
  gpio_set_outover(root->pin_dm,  GPIO_OVERRIDE_NORMAL);
  busy_wait_us(100); // TODO check if this is neccessary

  // bus back to operating
  root->suspended = false;
}

void pio_usb_host_close_device(uint8_t root_idx, uint8_t device_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    if ((ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
        ep->size) {
#if PIO_USB_HOST_BULK_STREAM
      if (ep->bulk_ring && !pio_usb_host_bulk_stream_stop(ep->bulk_ring, 10000)) {
        // The frame core may still be using the ring. Leave the endpoint
        // open; the ring's owner keeps the storage and retries its stop.
        continue;
      }
#endif
      ep->size = 0;
      ep->has_transfer = false;
#if PIO_USB_HOST_ISOCHRONOUS
      if (ep == iso_ring_ep) {
        iso_ring_ep = NULL;
      }
#endif
    }
  }
}

static inline __force_inline endpoint_t * _find_ep(uint8_t root_idx, 
                                                   uint8_t device_address, uint8_t ep_address) {
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // note 0x00 and 0x80 are matched as control endpoint of opposite direction
    if ((ep->root_idx == root_idx) && (ep->dev_addr == device_address) &&
        ep->size &&
        ((ep->ep_num == ep_address) ||
         (((ep_address & 0x7f) == 0) && ((ep->ep_num & 0x7f) == 0)))) {
      return ep;
    }
  }

  return NULL;
}

bool pio_usb_host_endpoint_open(uint8_t root_idx, uint8_t device_address,
                                uint8_t const *desc_endpoint, bool need_pre) {
  const endpoint_descriptor_t *d = (const endpoint_descriptor_t *)desc_endpoint;
  if (NULL != _find_ep(root_idx, device_address, d->epaddr)) {
    return true; // already opened
  }
  for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
    endpoint_t *ep = PIO_USB_ENDPOINT(ep_pool_idx);
    // ep size is used as valid indicator
    if (ep->size == 0) {
      pio_usb_ll_configure_endpoint(ep, desc_endpoint);
      ep->root_idx = root_idx;
      ep->dev_addr = device_address;
      ep->need_pre = need_pre;
      ep->is_tx = (d->epaddr & 0x80) ? false : true; // host endpoint out is tx
      return true;
    }
  }

  return false;
}

bool pio_usb_host_endpoint_close(uint8_t root_idx, uint8_t device_address,
                                 uint8_t ep_address) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    return false; // endpoint not opened
  }

#if PIO_USB_HOST_BULK_STREAM
  if (ep->bulk_ring && !pio_usb_host_bulk_stream_stop(ep->bulk_ring, 10000)) {
    return false; // still streaming; the ring's owner must stop it first
  }
#endif
  ep->size = 0; // mark as closed
#if PIO_USB_HOST_ISOCHRONOUS
  if (ep == iso_ring_ep) {
    iso_ring_ep = NULL;
  }
#endif
  return true;
}

bool pio_usb_host_send_setup(uint8_t root_idx, uint8_t device_address,
                             uint8_t const setup_packet[8]) {
  endpoint_t *ep = _find_ep(root_idx, device_address, 0);
  if (!ep) {
    printf("cannot find ep 0x00\r\n");
    return false;
  }

  ep->ep_num = 0; // setup is is OUT
  ep->data_id = USB_PID_SETUP;
  ep->is_tx = true;

#if PIO_USB_HOST_ISOCHRONOUS || PIO_USB_HOST_BULK_STREAM
  // SET_CONFIGURATION or SET_INTERFACE: the device's endpoints may change
  // or disappear, so stop streaming from them.
  bool const reconfigures = (setup_packet[0] & 0x60) == 0 &&
                            (setup_packet[1] == 9 || setup_packet[1] == 11);
#endif
#if PIO_USB_HOST_ISOCHRONOUS
  if (reconfigures && iso_ring_ep && iso_ring_ep->dev_addr == device_address) {
    iso_ring_ep = NULL;
  }
#endif
#if PIO_USB_HOST_BULK_STREAM
  if (reconfigures) {
    // Only a request: the frame core detaches on its next frame, before it
    // sends this SETUP. The owner still has to stop() before freeing.
    for (int ep_pool_idx = 0; ep_pool_idx < PIO_USB_EP_POOL_CNT; ep_pool_idx++) {
      endpoint_t *stream_ep = PIO_USB_ENDPOINT(ep_pool_idx);
      pio_usb_bulk_ring_t *ring = stream_ep->bulk_ring;
      if (ring && stream_ep->root_idx == root_idx && stream_ep->dev_addr == device_address) {
        ring->stop_requested = true;
      }
    }
  }
#endif

  return pio_usb_ll_transfer_start(ep, (uint8_t *)setup_packet, 8);
}

bool pio_usb_host_endpoint_transfer(uint8_t root_idx, uint8_t device_address,
                                    uint8_t ep_address, uint8_t *buffer,
                                    uint16_t buflen) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    printf("no endpoint 0x%02X\r\n", ep_address);
    return false;
  }

#if PIO_USB_HOST_BULK_STREAM
  if (ep->bulk_ring) {
    return false; // the stream owns this endpoint until it is stopped
  }
#endif

  // Control endpoint, address may switch between 0x00 <-> 0x80
  // therefore we need to update ep_num and is_tx
  if ((ep_address & 0x7f) == 0) {
    ep->ep_num = ep_address;
    ep->is_tx = ep_address == 0;
    ep->data_id = 1; // data and status always start with DATA1
  }

#if PIO_USB_HOST_ISOCHRONOUS
  if (iso_ring != NULL &&
      (ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS && !ep->is_tx && ep != iso_ring_ep) {
    // Start receiving this endpoint's packets every frame. Only one
    // isochronous IN endpoint at a time uses the ring.
    iso_ring_ep = NULL;
    iso_ring_head = 0;
    iso_ring_count = 0;
    iso_ring_lost = false;
    iso_ring_silent = 0;
    iso_ring_ep = ep;
  }
#endif

  return pio_usb_ll_transfer_start(ep, buffer, buflen);
}

bool pio_usb_host_endpoint_abort_transfer(uint8_t root_idx, uint8_t device_address,
                                          uint8_t ep_address) {
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  if (!ep) {
    printf("no endpoint 0x%02X\r\n", ep_address);
    return false;
  }

  if (!ep->has_transfer) {
    return false; // no transfer to abort
  }

  // mark transfer as aborted
  ep->transfer_aborted = true;

  // Race potential: SOF timer can be called before transfer_aborted is actually set
  // and started the transfer. Wait 1 usb frame for transaction to complete.
  // On the next SOF timer, transfer_aborted will be checked and skipped
  for (int wait_ms = 0; wait_ms < 8 && ep->has_transfer && ep->transfer_started; wait_ms++) {
    busy_wait_ms(1);
  }

  // check if transfer is still active (could be completed)
  bool const still_active = ep->has_transfer;
  if (still_active) {
    ep->has_transfer = false;
  }
  ep->transfer_aborted = false;

  return still_active; // still active means transfer is successfully aborted
}

#if PIO_USB_HOST_BULK_STREAM
//--------------------------------------------------------------------+
// Bulk stream ring, consumer side. Called from the application, never
// from the host frame handler.
//--------------------------------------------------------------------+
bool pio_usb_host_bulk_stream_start(uint8_t root_idx, uint8_t device_address,
                                    uint8_t ep_address, pio_usb_bulk_ring_t *ring,
                                    uint8_t *storage, uint32_t capacity) {
  if (!ring || !storage || ring->active || capacity < BULK_STREAM_MIN_CAPACITY ||
      capacity > BULK_STREAM_MAX_CAPACITY || (capacity & (capacity - 1)) ||
      root_idx >= PIO_USB_ROOT_PORT_CNT) {
    return false;
  }
  endpoint_t *ep = _find_ep(root_idx, device_address, ep_address);
  root_port_t *root = PIO_USB_ROOT_PORT(root_idx);
  if (!ep || ep->has_transfer || ep->bulk_ring || ep->transfer_started ||
      !(ep->ep_num & EP_IN) || (ep->attr & 0x03) != EP_ATTR_BULK ||
      ep->size > PIO_USB_EP_SIZE || ep->need_pre || !root->is_fullspeed ||
      !root->connected || root->suspended) {
    return false;
  }
#if PIO_USB_HOST_ISOCHRONOUS
  if (ep == iso_ring_ep) {
    return false; // cannot happen for a bulk endpoint; keep the rings apart
  }
#endif
  memset(ring, 0, sizeof(*ring));
  ring->buffer = storage;
  ring->capacity = capacity;
  ring->active = true;
  ep->interval_counter = 0;
  ep->transfer_aborted = false;
  ep->failed_count = 0;
  __dmb(); // the frame core must see a fully initialised ring
  ep->bulk_ring = ring;
  return true;
}

uint32_t pio_usb_host_bulk_stream_read(pio_usb_bulk_ring_t *ring, uint8_t *dest,
                                       uint32_t len) {
  if (!ring || !dest || !len) {
    return 0;
  }
  uint32_t const read = ring->read_pos;
  uint32_t const written = ring->write_pos;
  __dmb();
  uint32_t available = written - read;
  if (available > ring->capacity) {
    return 0; // inconsistent state; never read outside the storage
  }
  if (len > available) {
    len = available;
  }
  uint32_t const offset = read & (ring->capacity - 1);
  uint32_t first = ring->capacity - offset;
  if (first > len) {
    first = len;
  }
  if (first) {
    memcpy(dest, ring->buffer + offset, first);
  }
  if (len > first) {
    memcpy(dest + first, ring->buffer, len - first);
  }
  __dmb(); // the producer may reuse these bytes only after the copy is done
  ring->read_pos = read + len;
  return len;
}

bool pio_usb_host_bulk_stream_stop(pio_usb_bulk_ring_t *ring, uint32_t timeout_us) {
  if (!ring) {
    return true;
  }
  ring->stop_requested = true;
  __dmb();
  uint32_t const started = get_time_us_32();
  while (ring->active) {
    if (get_time_us_32() - started >= timeout_us) {
      return false;
    }
    tight_loop_contents();
  }
  __dmb();
  return true;
}

bool pio_usb_host_bulk_stream_stats(pio_usb_bulk_ring_t *ring, pio_usb_bulk_stats_t *out) {
  if (!ring || !out) {
    return false;
  }
  // A transaction can hold the seqlock for hundreds of microseconds, so
  // bound the retry by time rather than by a spin count.
  uint32_t const started = get_time_us_32();
  while (get_time_us_32() - started < 2000) {
    uint32_t const before = ring->stats_seq;
    __dmb();
    if (before & 1) {
      continue;
    }
    *out = ring->stats;
    __dmb();
    if (before != ring->stats_seq) {
      continue;
    }
    uint32_t const written = ring->write_pos;
    uint32_t const read = ring->read_pos;
    __dmb();
    out->available_bytes = written - read;
    if (out->available_bytes > ring->capacity) {
      out->available_bytes = 0;
    }
    out->active = ring->active;
    return true;
  }
  return false;
}
#endif // PIO_USB_HOST_BULK_STREAM

//--------------------------------------------------------------------+
// Transaction helper
//--------------------------------------------------------------------+
static void __no_inline_not_in_flash_func(end_transaction)(pio_port_t *pp) {
  pio_sm_set_enabled(pp->pio_usb_rx, pp->sm_rx, false);
  if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
    pio_sm_exec(pp->pio_usb_rx, pp->sm_eop, pio_encode_jmp(pp->offset_eop));
  }
}

static int __no_inline_not_in_flash_func(usb_in_transaction)(pio_port_t *pp,
                                                             endpoint_t *ep) {
  int res = 0;
  uint8_t expect_pid = (ep->data_id == 1) ? USB_PID_DATA1 : USB_PID_DATA0;

  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_send_token(pp, USB_PID_IN, ep->dev_addr, ep->ep_num);
  pio_usb_bus_start_receive(pp);

  bool const is_iso =
      PIO_USB_HOST_ISOCHRONOUS && (ep->attr & 0x03) == EP_ATTR_ISOCHRONOUS;
  // Isochronous IN has no handshake and no data toggle
  int receive_len = pio_usb_bus_receive_packet_and_handshake(pp, is_iso ? 0 : USB_PID_ACK);
  uint8_t const receive_pid = pp->usb_rx_buffer[1];

  if (is_iso && receive_len > 0) {
    // Clamp to what usb_rx_buffer holds. A non-compliant or malicious device
    // could otherwise report a receive_len larger than the buffer, causing an
    // out-of-bounds memcpy.
    if ((uint16_t)receive_len > sizeof(pp->usb_rx_buffer) - 4) {
      receive_len = sizeof(pp->usb_rx_buffer) - 4;
    }
  }

#if PIO_USB_HOST_ISOCHRONOUS
  if (is_iso && iso_ring != NULL) {
    if (ep == iso_ring_ep) {
      iso_ring_receive(ep, receive_len, receive_pid, &pp->usb_rx_buffer[2]);
    }
  } else
#endif
  if (is_iso) {
    // Every isochronous packet completes the transfer, even a full one, so
    // the caller sees packet boundaries. A bad or missing packet is not retried.
    if (receive_len >= 0) {
      uint16_t const room = ep->total_len - ep->actual_len;
      if (receive_len > room) {
        receive_len = room;
      }
      memcpy(ep->app_buf, &pp->usb_rx_buffer[2], receive_len);
      ep->actual_len += receive_len;
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
    } else {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
  } else if (receive_len >= 0) {
    if (receive_pid == expect_pid) {
      // Clamp to the transaction length the host actually requested/allocated
      // a buffer for. A non-compliant or malicious device could otherwise
      // report a receive_len larger than ep->app_buf (or usb_rx_buffer),
      // causing an out-of-bounds memcpy.
      uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);
      if ((uint16_t)receive_len > xact_len) {
        receive_len = xact_len;
      }
      memcpy(ep->app_buf, &pp->usb_rx_buffer[2], receive_len);
      pio_usb_ll_transfer_continue(ep, receive_len);
    } else {
      // DATA0/1 mismatched, 0 for re-try next frame
    }
  } else if (receive_pid == USB_PID_NAK) {
    // NAK try again next frame
  } else if (receive_pid == USB_PID_STALL) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_STALLED_BITS);
  } else {
    res = -1;
    if ((pp->pio_usb_rx->irq & IRQ_RX_COMP_MASK) == 0) {
      res = -2;
    }

    if (++ep->failed_count >= TRANSACTION_MAX_RETRY) {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS); // failed after 3 consecutive retries
    }
  }

  if (res == 0) {
    ep->failed_count = 0; // reset failed count if we got a sound response
  }

  end_transaction(pp);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

static int __no_inline_not_in_flash_func(usb_out_transaction)(pio_port_t *pp,
                                                              endpoint_t *ep) {
  int res = 0;

  uint16_t const xact_len = pio_usb_ll_get_transaction_len(ep);

  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_send_token(pp, USB_PID_OUT, ep->dev_addr, ep->ep_num);

  pio_usb_bus_usb_transfer(pp, ep->buffer, ep->encoded_data_len);
  pio_usb_bus_start_receive(pp);

  pio_usb_bus_wait_handshake(pp);
  end_transaction(pp);

  uint8_t const receive_token = pp->usb_rx_buffer[1];

  if (receive_token == USB_PID_ACK) {
    pio_usb_ll_transfer_continue(ep, xact_len);
  } else if (receive_token == USB_PID_NAK) {
    // NAK try again next frame
  } else if (receive_token == USB_PID_STALL) {
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_STALLED_BITS);
  } else {
    res = -1;
    if (++ep->failed_count >= TRANSACTION_MAX_RETRY) {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
  }

  if (res == 0) {
    ep->failed_count = 0;// reset failed count if we got a sound response
  }

  end_transaction(pp);
  pp->usb_rx_buffer[0] = 0;
  pp->usb_rx_buffer[1] = 0;

  return res;
}

static int __no_inline_not_in_flash_func(usb_setup_transaction)(
    pio_port_t *pp,  endpoint_t *ep) {
  int res = 0;

  // Setup token
  pio_usb_bus_prepare_receive(pp);
  pio_usb_bus_send_token(pp, USB_PID_SETUP, ep->dev_addr, 0);

  // Data
  ep->data_id = 0; // set to DATA0
  pio_usb_bus_usb_transfer(pp, ep->buffer, ep->encoded_data_len);

  // Handshake
  pio_usb_bus_start_receive(pp);
  const uint8_t handshake = pio_usb_bus_wait_handshake(pp);
  end_transaction(pp);

  if (handshake == USB_PID_ACK) {
    ep->actual_len = 8;
    pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS);
  } else {
    res = -1;
    ep->data_id = USB_PID_SETUP; // retry setup
    if (++ep->failed_count >= TRANSACTION_MAX_RETRY) {
      pio_usb_ll_transfer_complete(ep, PIO_USB_INTS_ENDPOINT_ERROR_BITS);
    }
  }

  if (res == 0) {
    ep->failed_count = 0;// reset failed count if we got a sound response
  }

  pp->usb_rx_buffer[1] = 0; // reset buffer

  return res;
}


static void __no_inline_not_in_flash_func(handle_endpoint_irq)(
    root_port_t *root, uint32_t flag, volatile uint32_t *ep_reg) {
  (void)root;
  const uint32_t ep_all = *ep_reg;

  for (uint8_t ep_idx = 0; ep_idx < PIO_USB_EP_POOL_CNT; ep_idx++) {
    if (ep_all & (1u << ep_idx)) {
      endpoint_t *ep = PIO_USB_ENDPOINT(ep_idx);
      usb_device_t *device = NULL;

      // find device this endpoint belongs to
      for (int idx = 0; idx < PIO_USB_DEVICE_CNT; idx++) {
        usb_device_t *dev = &pio_usb_device[idx];
        if (dev->connected && (ep->dev_addr == dev->address)) {
          device = dev;
          break;
        }
      }

      if (device) {
        // control endpoint is either 0x00 or 0x80
        if ((ep->ep_num & 0x7f) == 0) {
          control_pipe_t *pipe = &device->control_pipe;

          if (flag != PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
            pipe->stage = STAGE_SETUP;
            pipe->operation = CONTROL_ERROR;
          } else {
            ep->data_id = 1; // both data and status have DATA1
            if (pipe->stage == STAGE_SETUP) {
              if (pipe->operation == CONTROL_IN) {
                pipe->stage = STAGE_IN;
                ep->ep_num = 0x80;
                ep->is_tx = false;
                pio_usb_ll_transfer_start(ep,
                                          (uint8_t *)(uintptr_t)pipe->rx_buffer,
                                          pipe->request_length);
              } else if (pipe->operation == CONTROL_OUT) {
                if (pipe->out_data_packet.tx_address != NULL) {
                  pipe->stage = STAGE_OUT;
                  ep->ep_num = 0x00;
                  ep->is_tx = true;
                  pio_usb_ll_transfer_start(ep,
                                            pipe->out_data_packet.tx_address,
                                            pipe->out_data_packet.tx_length);
                } else {
                  pipe->stage = STAGE_STATUS;
                  ep->ep_num = 0x80;
                  ep->is_tx = false;
                  pio_usb_ll_transfer_start(ep, NULL, 0);
                }
              }
            } else if (pipe->stage == STAGE_IN) {
              pipe->stage = STAGE_STATUS;
              ep->ep_num = 0x00;
              ep->is_tx = true;
              pio_usb_ll_transfer_start(ep, NULL, 0);
            } else if (pipe->stage == STAGE_OUT) {
              pipe->stage = STAGE_STATUS;
              ep->ep_num = 0x80;
              ep->is_tx = false;
              pio_usb_ll_transfer_start(ep, NULL, 0);
            } else if (pipe->stage == STAGE_STATUS) {
              pipe->stage = STAGE_SETUP;
              pipe->operation = CONTROL_COMPLETE;
            }
          }
        } else if (device->device_class == CLASS_HUB && (ep->ep_num & EP_IN)) {
          // hub interrupt endpoint
          device->event = EVENT_HUB_PORT_CHANGE;
        }
      }
    }
  }

  // clear all
  (*ep_reg) &= ~ep_all;
}

// IRQ Handler
static void __no_inline_not_in_flash_func(__pio_usb_host_irq_handler)(uint8_t root_id) {
  root_port_t *root = PIO_USB_ROOT_PORT(root_id);
  uint32_t const ints = root->ints;

  if (ints & PIO_USB_INTS_CONNECT_BITS) {
    root->event = EVENT_CONNECT;
  }

  if (ints & PIO_USB_INTS_DISCONNECT_BITS) {
    root->event = EVENT_DISCONNECT;
  }

  if (ints & PIO_USB_INTS_ENDPOINT_COMPLETE_BITS) {
    handle_endpoint_irq(root, PIO_USB_INTS_ENDPOINT_COMPLETE_BITS,
                        &root->ep_complete);
  }

  if (ints & PIO_USB_INTS_ENDPOINT_STALLED_BITS) {
    handle_endpoint_irq(root, PIO_USB_INTS_ENDPOINT_STALLED_BITS,
                        &root->ep_stalled);
  }

  if (ints & PIO_USB_INTS_ENDPOINT_ERROR_BITS) {
    handle_endpoint_irq(root, PIO_USB_INTS_ENDPOINT_ERROR_BITS,
                        &root->ep_error);
  }

  // clear all
  root->ints &= ~ints;
}

// weak alias to __pio_usb_host_irq_handler
void pio_usb_host_irq_handler(uint8_t root_id) __attribute__ ((weak, alias("__pio_usb_host_irq_handler")));

#pragma GCC pop_options

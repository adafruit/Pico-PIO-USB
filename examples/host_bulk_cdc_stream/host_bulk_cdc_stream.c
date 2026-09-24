/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2026 Tim Cocks (Adafruit Industries)
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

// This example demonstrates the host bulk IN streaming support of
// Pico-PIO-USB (PIO_USB_HOST_BULK_STREAM).
//
// A full-speed USB CDC serial device is plugged into the PIO USB host port.
// The companion firmware in sender/ turns any RP2040/RP2350 board into such
// a device: once the host raises DTR it sends a little-endian 32-bit counter
// starting at 0 as fast as it can.
//
// The example finds the device's CDC data interface, opens its bulk IN
// endpoint with the TinyUSB bare endpoint API, raises DTR and attaches a
// ring buffer to the endpoint with pio_usb_host_bulk_stream_start(). From
// then on the host frame handler (core1) polls the endpoint every frame, plus
// extra packets in the idle rest of the frame, and appends the payload to
// the ring. No transfer is queued and no TinyUSB callback runs per packet.
//
// core0 is the consumer. It drains the ring with
// pio_usb_host_bulk_stream_read(), checks that the counter never skips, and
// once a second prints throughput and counters from
// pio_usb_host_bulk_stream_stats() over the native USB CDC port.
//
// Terminal keys:
//   'p'  pause/resume the consumer. The host keeps receiving; once the ring
//        is full new packets are dropped and counted as overruns, and the
//        counter check reports the gap on resume.
//   's'  stop/restart the stream with pio_usb_host_bulk_stream_stop() and
//        _start(). While stopped the host sends no IN tokens and the device
//        holds its data, so the counter continues without a gap after the
//        restart.
//   'd'  dump every field of pio_usb_bulk_stats_t.
// Opening the terminal at 1200 baud reboots into the UF2 bootloader.
//
// Any other CDC device works too (USB serial adapters, CircuitPython and
// Arduino boards); the counter check then just reports its data as errors.
//
// For TinyUSB roothub port0 is native usb controller, roothub port1 is
// pico-pio-usb.

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/sync.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "pio_usb.h"
#include "tusb.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// CDC constants (USB Class Definitions for Communications Devices 1.2)
#define CDC_SUBCLASS_ACM 0x02
#define CDC_REQ_SET_CONTROL_LINE_STATE 0x22
#define CDC_LINE_STATE_DTR 0x01
#define CDC_LINE_STATE_RTS 0x02

// Ring the host writes the stream into. Must be a power of two in
// 4096..65536 and live in internal SRAM, which .bss does.
#ifndef BULK_RING_SIZE
#define BULK_RING_SIZE 32768
#endif

// Pico-PIO-USB root port of the host port. TinyUSB roothub port 1 is
// Pico-PIO-USB root port 0.
#define BULK_ROOT_IDX 0

// Bytes copied out of the ring per read
#define READ_CHUNK_SIZE 4096

// D+ pin of the PIO USB host port (D- is the next pin). Boards in the pico-sdk
// with a PIO USB host port define PICO_DEFAULT_PIO_USB_DP_PIN.
#ifndef PIO_USB_DP_PIN
#ifdef PICO_DEFAULT_PIO_USB_DP_PIN
#define PIO_USB_DP_PIN PICO_DEFAULT_PIO_USB_DP_PIN
#else
#define PIO_USB_DP_PIN PIO_USB_DP_PIN_DEFAULT
#endif
#endif

// Optional active-high enable for the host port's 5V supply.
#ifndef PIO_USB_VBUSEN_PIN
#ifdef ADAFRUIT_FRUIT_JAM_USB_HOST_5V_POWER_PIN
#define PIO_USB_VBUSEN_PIN ADAFRUIT_FRUIT_JAM_USB_HOST_5V_POWER_PIN
#endif
#endif

// The device found by core1. Written by core1 before it bumps ready_gen,
// read by core0 after it sees ready_gen change.
typedef struct {
  uint8_t daddr;
  uint8_t comm_itf; // 0xff if the device has no CDC communication interface
  uint8_t data_itf;
  tusb_desc_endpoint_t ep_desc;
} cdc_info_t;

// Counter check state, core0 only. The stream is a sequence of
// little-endian 32-bit words, each one more than the previous one.
typedef struct {
  bool synced;       // expected is valid
  bool ever_synced;  // at least one sync since the stream (re)started
  uint32_t expected; // next word
  uint32_t word;     // word being assembled
  uint8_t word_len;  // bytes in word
  uint64_t window;   // last 8 bytes seen while searching for sync
  uint8_t window_len;
  uint32_t first;    // first word after the initial sync
  uint32_t words;    // words that matched
  uint32_t errors;   // sync losses
  uint32_t skipped;  // bytes discarded while searching for sync
  int64_t lost;      // bytes missing across gaps, negative for repeats
} checker_t;

typedef enum {
  STREAM_IDLE,    // no device, or the stream ended by itself
  STREAM_RUNNING, // ring attached, consumer draining it
  STREAM_STOPPED, // stopped with 's', waiting for 's' again
} stream_state_t;

static cdc_info_t cdc;
static volatile uint32_t ready_gen;   // core1 -> core0: bumped when cdc is ready
static volatile bool device_present;  // core1 -> core0: cleared on unmount

static volatile bool pause_request;   // tud_cdc_rx_cb -> core0 loop
static volatile bool stop_request;
static volatile bool dump_request;

// Messages from core1 (host) to core0 (printing). Only constant strings are
// queued, so a full queue just drops the newest message.
#define MSG_QUEUE_LEN 8
static const char *msg_queue[MSG_QUEUE_LEN];
static volatile uint8_t msg_head, msg_tail;

static void post_msg(const char *msg) {
  uint8_t const next = (uint8_t)((msg_tail + 1) % MSG_QUEUE_LEN);
  if (next != msg_head) {
    msg_queue[msg_tail] = msg;
    msg_tail = next;
  }
}

CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN static uint8_t cfg_desc_buf[CFG_TUH_ENUMERATION_BUFSIZE];

// The ring descriptor and its storage stay owned by the application. Both
// are static here, so they are never freed while the frame core uses them.
static pio_usb_bulk_ring_t ring;
static uint8_t ring_storage[BULK_RING_SIZE] __attribute__((aligned(4)));
static uint8_t read_buf[READ_CHUNK_SIZE];

static void consumer_task(void);
static void print_task(void);

/*------------- MAIN -------------*/

// core1: handle host events. The Pico-PIO-USB frame handler runs in a timer
// interrupt on this core, so this is the core that fills the ring.
void core1_main() {
  sleep_ms(10);

  // Use tuh_configure() to pass pio configuration to the host stack
  // Note: tuh_configure() must be called before
  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIO_USB_DP_PIN;
  tuh_configure(1, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
  // port1) on core1
  tuh_init(1);

  while (true) {
    tuh_task(); // tinyusb host task
  }
}

// core0: handle device events and consume the stream
int main(void) {
  // default 125MHz is not appropreate. Sysclock should be multiple of 12MHz.
  set_sys_clock_khz(120000, true);

  sleep_ms(10);

#ifdef PIO_USB_VBUSEN_PIN
  // Power the host port
  gpio_init(PIO_USB_VBUSEN_PIN);
  gpio_set_dir(PIO_USB_VBUSEN_PIN, GPIO_OUT);
  gpio_put(PIO_USB_VBUSEN_PIN, 1);
#endif

  multicore_reset_core1();
  // all USB host task run in core1
  multicore_launch_core1(core1_main);

  // init device stack on native usb (roothub port0)
  tud_init(0);

  while (true) {
    tud_task(); // tinyusb device task
    consumer_task();
    print_task();
    tud_cdc_write_flush();
  }

  return 0;
}

//--------------------------------------------------------------------+
// Device CDC
//--------------------------------------------------------------------+

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf) {
  (void)itf;

  char buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));
  for (uint32_t i = 0; i < count; i++) {
    switch (buf[i]) {
    case 'p': case 'P': pause_request = true; break;
    case 's': case 'S': stop_request = true; break;
    case 'd': case 'D': dump_request = true; break;
    default: break;
    }
  }
}

// Invoked when the host sets the line coding. 1200 baud is the usual
// request to reboot into the bootloader for flashing.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding) {
  (void)itf;

  if (p_line_coding->bit_rate == 1200) {
    reset_usb_boot(0, 0);
  }
}

static void cdc_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void cdc_printf(const char *fmt, ...) {
  if (!tud_cdc_connected()) {
    return;
  }
  char tempbuf[200];
  va_list args;
  va_start(args, fmt);
  int count = vsnprintf(tempbuf, sizeof(tempbuf), fmt, args);
  va_end(args);
  if (count > 0) {
    if (count > (int)sizeof(tempbuf)) {
      count = sizeof(tempbuf);
    }
    tud_cdc_write(tempbuf, count);
  }
}

//--------------------------------------------------------------------+
// Counter check
//--------------------------------------------------------------------+

static checker_t check;

static void checker_reset(void) {
  memset(&check, 0, sizeof(check));
}

// The checker is not reset when the stream is restarted with 's', so a
// restart that loses or repeats data shows up as a sync loss.
static void checker_feed(uint8_t const *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    uint8_t const b = data[i];
    if (check.synced) {
      check.word |= (uint32_t)b << (8 * check.word_len);
      if (++check.word_len < 4) {
        continue;
      }
      uint32_t const word = check.word;
      check.word = 0;
      check.word_len = 0;
      if (word == check.expected) {
        check.expected++;
        check.words++;
        continue;
      }
      // Lost or repeated data: search for two consecutive words again,
      // starting with the bytes of the word that did not match.
      check.synced = false;
      check.errors++;
      check.window = (uint64_t)word << 32;
      check.window_len = 4;
      continue;
    }

    check.window = (check.window >> 8) | ((uint64_t)b << 56);
    if (check.window_len < 8) {
      check.window_len++;
    }
    if (check.window_len < 8) {
      continue;
    }
    uint32_t const lo = (uint32_t)check.window;
    uint32_t const hi = (uint32_t)(check.window >> 32);
    if (hi != lo + 1) {
      check.skipped++;
      continue;
    }
    if (check.ever_synced) {
      check.lost += 4 * (int64_t)(int32_t)(lo - check.expected);
    } else {
      check.first = lo;
      check.ever_synced = true;
    }
    check.words += 2;
    check.expected = hi + 1;
    check.synced = true;
    check.window_len = 0;
  }
}

//--------------------------------------------------------------------+
// Consumer (core0)
//--------------------------------------------------------------------+

static stream_state_t state;
static uint32_t started_gen;
static bool paused;

// Snapshot at the previous once-a-second report
static pio_usb_bulk_stats_t last_stats;
static uint32_t last_errors;
static uint32_t last_report_ms;

static void dump_stats(pio_usb_bulk_stats_t const *s) {
  cdc_printf("  polls %lu packets %lu bytes %lu zero %lu errors %lu\r\n",
             (unsigned long)s->polls, (unsigned long)s->packets, (unsigned long)s->bytes,
             (unsigned long)s->zero_packets, (unsigned long)s->error_packets);
  cdc_printf("  overrun %lu packets / %lu bytes, ring high water %lu of %u, waiting %lu\r\n",
             (unsigned long)s->overrun_packets, (unsigned long)s->overrun_bytes,
             (unsigned long)s->high_water_bytes, BULK_RING_SIZE,
             (unsigned long)s->available_bytes);
  cdc_printf("  frames %lu active, %lu unarmed; max frame %lu us, %lu frame overruns\r\n",
             (unsigned long)s->active_frames, (unsigned long)s->unarmed_frames,
             (unsigned long)s->max_frame_us, (unsigned long)s->frame_overruns);
  cdc_printf("  bus time %lu us total, %lu us max per transaction, max poll gap %lu us\r\n",
             (unsigned long)s->total_transaction_us, (unsigned long)s->max_transaction_us,
             (unsigned long)s->max_poll_gap_us);
  cdc_printf("  active %lu disconnected %lu\r\n", (unsigned long)s->active,
             (unsigned long)s->disconnected);
  cdc_printf("  counter: %lu words ok, first 0x%08lx, next 0x%08lx, %lu sync losses, "
             "%ld bytes lost, %lu bytes skipped\r\n",
             (unsigned long)check.words, (unsigned long)check.first,
             (unsigned long)check.expected, (unsigned long)check.errors, (long)check.lost,
             (unsigned long)check.skipped);
}

// Read and check everything waiting in the ring
static void drain(void) {
  uint32_t n;
  while ((n = pio_usb_host_bulk_stream_read(&ring, read_buf, sizeof(read_buf))) > 0) {
    checker_feed(read_buf, n);
  }
}

static bool start_stream(void) {
  // start() clears the ring and its counters
  if (!pio_usb_host_bulk_stream_start(BULK_ROOT_IDX, cdc.daddr, cdc.ep_desc.bEndpointAddress,
                                      &ring, ring_storage, sizeof(ring_storage))) {
    return false;
  }
  memset(&last_stats, 0, sizeof(last_stats));
  last_errors = check.errors;
  last_report_ms = to_ms_since_boot(get_absolute_time());
  state = STREAM_RUNNING;
  return true;
}

// The frame core detached the ring: the device went away, STALLed the
// endpoint, or TinyUSB closed it.
static void stream_ended(void) {
  // Returns at once: the ring is already detached
  pio_usb_host_bulk_stream_stop(&ring, 10000);
  drain();
  pio_usb_bulk_stats_t s;
  bool const have_stats = pio_usb_host_bulk_stream_stats(&ring, &s);
  // TinyUSB runs tuh_umount_cb() before it closes the endpoints, so an
  // unplug is already visible here. The disconnected counter is only set
  // when the frame handler itself sees the port go away first.
  bool const unplugged = !device_present || (have_stats && s.disconnected);
  cdc_printf("Stream ended (%s)\r\n",
             unplugged ? "device disconnected" : "endpoint stalled or closed");
  if (have_stats) {
    dump_stats(&s);
  }
  state = STREAM_IDLE;
}

static void report(void) {
  pio_usb_bulk_stats_t s;
  if (!pio_usb_host_bulk_stream_stats(&ring, &s)) {
    cdc_printf("stats unavailable: the host frame handler is not running\r\n");
    return;
  }
  uint32_t const polls = s.polls - last_stats.polls;
  uint32_t const packets = s.packets - last_stats.packets;
  uint32_t const errors = s.error_packets - last_stats.error_packets;
  uint32_t const naks = polls - packets - errors; // NAKs and duplicates
  cdc_printf("%4lu KB/s %5lu pkt/s %5lu nak/s  err %lu  overrun %lu  ring %5lu/%u  "
             "check %s%s\r\n",
             (unsigned long)((s.bytes - last_stats.bytes) / 1000), (unsigned long)packets,
             (unsigned long)naks, (unsigned long)errors,
             (unsigned long)(s.overrun_packets - last_stats.overrun_packets),
             (unsigned long)s.available_bytes, BULK_RING_SIZE,
             check.errors == last_errors ? (check.synced ? "ok" : "no sync")
                                         : "SKIP",
             paused ? "  (consumer paused)" : "");
  if (check.errors != last_errors) {
    cdc_printf("  counter skipped %lu time(s), %ld bytes lost in total\r\n",
               (unsigned long)(check.errors - last_errors), (long)check.lost);
  }
  last_stats = s;
  last_errors = check.errors;
}

static void consumer_task(void) {
  bool const do_pause = pause_request;
  bool const do_stop = stop_request;
  pause_request = false;
  stop_request = false;

  if (do_pause) {
    paused = !paused;
    cdc_printf(paused ? "Consumer paused: the ring fills, then packets are dropped\r\n"
                      : "Consumer resumed\r\n");
  }

  switch (state) {
  case STREAM_IDLE:
    if (device_present && ready_gen != started_gen) {
      started_gen = ready_gen;
      __dmb(); // read cdc only after seeing the new generation
      checker_reset();
      if (start_stream()) {
        cdc_printf("Streaming from endpoint 0x%02x into a %u byte ring; "
                   "'p' pause, 's' stop/start, 'd' details\r\n",
                   cdc.ep_desc.bEndpointAddress, BULK_RING_SIZE);
      } else {
        cdc_printf("pio_usb_host_bulk_stream_start() failed\r\n");
      }
    }
    return;

  case STREAM_STOPPED:
    if (!device_present || ready_gen != started_gen) {
      state = STREAM_IDLE; // unplugged or replaced while stopped
    } else if (do_stop) {
      if (start_stream()) {
        cdc_printf("Stream restarted\r\n");
      } else {
        cdc_printf("pio_usb_host_bulk_stream_start() failed\r\n");
      }
    }
    return;

  case STREAM_RUNNING:
    break;
  }

  if (do_stop) {
    if (!pio_usb_host_bulk_stream_stop(&ring, 10000)) {
      // The ring is still attached and must not be reused; try again
      cdc_printf("pio_usb_host_bulk_stream_stop() timed out\r\n");
      return;
    }
    // The ring keeps what was received before the stop
    drain();
    state = STREAM_STOPPED;
    cdc_printf("Stream stopped, ring drained; 's' restarts it\r\n");
    return;
  }

  if (!paused) {
    uint32_t const n = pio_usb_host_bulk_stream_read(&ring, read_buf, sizeof(read_buf));
    if (n) {
      checker_feed(read_buf, n);
    }
  }

  // active is cleared by the frame core as its last access to the ring
  if (!ring.active) {
    stream_ended();
    return;
  }

  uint32_t const now = to_ms_since_boot(get_absolute_time());
  if (dump_request) {
    dump_request = false;
    pio_usb_bulk_stats_t s;
    if (pio_usb_host_bulk_stream_stats(&ring, &s)) {
      dump_stats(&s);
    }
  }
  if (now - last_report_ms >= 1000) {
    last_report_ms = now;
    report();
  }
}

// core0: banner, messages from core1 and the waiting notice
static void print_task(void) {
  static bool was_connected;
  static uint32_t last_waiting_ms;

  // Nothing is printed until a terminal is attached, so the startup
  // messages are not lost.
  bool const connected = tud_cdc_connected();
  if (!connected) {
    was_connected = false;
    return;
  }
  if (!was_connected) {
    was_connected = true;
    cdc_printf("\r\nPico-PIO-USB bulk IN stream example, %u byte ring\r\n", BULK_RING_SIZE);
  }

  while (msg_head != msg_tail) {
    const char *msg = msg_queue[msg_head];
    msg_head = (uint8_t)((msg_head + 1) % MSG_QUEUE_LEN);
    cdc_printf("%s\r\n", msg);
  }

  uint32_t const now = to_ms_since_boot(get_absolute_time());
  if (state == STREAM_IDLE && !device_present && now - last_waiting_ms >= 5000) {
    last_waiting_ms = now;
    cdc_printf("Waiting for a USB CDC device on the PIO USB port...\r\n");
  }
}

//--------------------------------------------------------------------+
// Host: configuration descriptor parsing (core1)
//--------------------------------------------------------------------+

// Find the first CDC data interface with a bulk IN endpoint, and the CDC
// ACM communication interface before it, which receives the line state.
static bool find_cdc(uint8_t const *desc, uint16_t total_len, cdc_info_t *out) {
  uint8_t const *p = desc;
  uint8_t const *end = desc + total_len;
  uint8_t comm_itf = 0xff;
  bool in_data_itf = false;

  while (p + 2 <= end && p[0] >= 2 && p + p[0] <= end) {
    uint8_t const len = p[0];
    uint8_t const type = p[1];

    if (type == TUSB_DESC_INTERFACE && len >= sizeof(tusb_desc_interface_t)) {
      tusb_desc_interface_t const *itf = (tusb_desc_interface_t const *)p;
      in_data_itf = itf->bInterfaceClass == TUSB_CLASS_CDC_DATA;
      if (itf->bInterfaceClass == TUSB_CLASS_CDC &&
          itf->bInterfaceSubClass == CDC_SUBCLASS_ACM) {
        comm_itf = itf->bInterfaceNumber;
      }
      if (in_data_itf) {
        out->data_itf = itf->bInterfaceNumber;
      }
    } else if (in_data_itf && type == TUSB_DESC_ENDPOINT &&
               len >= sizeof(tusb_desc_endpoint_t)) {
      tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *)p;
      if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN &&
          ep->bmAttributes.xfer == TUSB_XFER_BULK) {
        memcpy(&out->ep_desc, p, sizeof(tusb_desc_endpoint_t));
        out->comm_itf = comm_itf;
        return true;
      }
    }

    p += len;
  }
  return false;
}

//--------------------------------------------------------------------+
// Host: setup (core1)
//--------------------------------------------------------------------+

// Hand the device to core0, which attaches the ring
static void cdc_ready(void) {
  __dmb(); // cdc must be visible before the new generation
  ready_gen++;
  post_msg("CDC device ready, raised DTR");
}

static void line_state_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    post_msg("Warning: SET_CONTROL_LINE_STATE failed, streaming anyway");
  }
  cdc_ready();
}

static void config_desc_cb(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    post_msg("Failed to get configuration descriptor");
    return;
  }

  tusb_desc_configuration_t const *cfg = (tusb_desc_configuration_t const *)cfg_desc_buf;
  uint16_t total_len = tu_le16toh(cfg->wTotalLength);
  if (total_len > sizeof(cfg_desc_buf)) {
    total_len = sizeof(cfg_desc_buf); // truncated: parse what we have
  }

  cdc_info_t found;
  memset(&found, 0, sizeof(found));
  if (!find_cdc(cfg_desc_buf, total_len, &found)) {
    post_msg("Device has no CDC data interface with a bulk IN endpoint");
    return;
  }
  if (tu_edpt_packet_size(&found.ep_desc) > 64) {
    post_msg("Bulk IN endpoint is larger than 64 bytes (not full speed)");
    return;
  }
  found.daddr = xfer->daddr;

  if (!tuh_edpt_open(found.daddr, &found.ep_desc)) {
    post_msg("Failed to open bulk IN endpoint");
    return;
  }
  cdc = found;
  device_present = true;

  if (cdc.comm_itf == 0xff) {
    cdc_ready();
    return;
  }

  // Most CDC devices, TinyUSB-based ones included, only send while DTR is
  // set. Until the ring is attached the device just NAKs the IN tokens that
  // are not being sent, so nothing is lost by raising it first.
  tusb_control_request_t const request = {
      .bmRequestType_bit = {.recipient = TUSB_REQ_RCPT_INTERFACE,
                            .type = TUSB_REQ_TYPE_CLASS,
                            .direction = TUSB_DIR_OUT},
      .bRequest = CDC_REQ_SET_CONTROL_LINE_STATE,
      .wValue = tu_htole16(CDC_LINE_STATE_DTR | CDC_LINE_STATE_RTS),
      .wIndex = tu_htole16(cdc.comm_itf),
      .wLength = 0,
  };
  tuh_xfer_t ctrl = {
      .daddr = cdc.daddr,
      .ep_addr = 0,
      .setup = &request,
      .buffer = NULL,
      .complete_cb = line_state_cb,
      .user_data = 0,
  };
  if (!tuh_control_xfer(&ctrl)) {
    post_msg("Warning: could not send SET_CONTROL_LINE_STATE");
    cdc_ready();
  }
}

//--------------------------------------------------------------------+
// Host: TinyUSB callbacks (core1)
//--------------------------------------------------------------------+

// Invoked when a device is mounted (configured)
void tuh_mount_cb(uint8_t daddr) {
  if (device_present) {
    post_msg("Another device mounted; already streaming from one CDC device");
    return;
  }
  if (!tuh_descriptor_get_configuration(daddr, 0, cfg_desc_buf, sizeof(cfg_desc_buf),
                                        config_desc_cb, 0)) {
    post_msg("Failed to request configuration descriptor");
  }
}

// Invoked when a device is unmounted. TinyUSB closes the device's endpoints;
// Pico-PIO-USB stops the stream first, and core0 sees the ring go inactive.
void tuh_umount_cb(uint8_t daddr) {
  if (device_present && daddr == cdc.daddr) {
    device_present = false;
    post_msg("CDC device unplugged");
  }
}

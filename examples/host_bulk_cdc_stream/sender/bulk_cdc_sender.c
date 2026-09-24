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

// Companion device firmware for the host_bulk_cdc_stream example.
//
// Runs on any RP2040/RP2350 board and uses its native USB port as a
// full-speed CDC ACM device. While the host holds DTR set, it sends a
// little-endian 32-bit counter over the bulk IN endpoint as fast as the
// host reads it. Every rising edge of DTR restarts the counter at 0.
// Data from the host is ignored.
//
// Build with -DBULK_SENDER_BYTES_PER_SEC=n to limit the rate, which makes
// the device NAK part of the host's polls.
//
// The board LED blinks while waiting for DTR and stays on while sending.
// Opening the port at 1200 baud reboots into the UF2 bootloader.

#include <string.h>

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "tusb.h"

#ifndef SENDER_BYTES_PER_SEC
#define SENDER_BYTES_PER_SEC 0
#endif

// Written in whole packets, so every packet starts on a word boundary
#define CHUNK_WORDS 16

static uint32_t counter;

#if SENDER_BYTES_PER_SEC
static uint64_t session_start_us;
static uint64_t sent; // bytes counted against the rate since session_start_us
#endif

static void led_task(void);
static void send_task(void);

int main(void) {
#ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

  // init device stack on native usb (roothub port0)
  tud_init(0);

  while (true) {
    tud_task(); // tinyusb device task
    send_task();
    led_task();
  }

  return 0;
}

// Invoked when the host changes DTR or RTS
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;
  (void)rts;

  if (dtr) {
    // New session: drop anything left from the previous one and start over
    tud_cdc_write_clear();
    counter = 0;
#if SENDER_BYTES_PER_SEC
    session_start_us = time_us_64();
    sent = 0;
#endif
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

// Keep the CDC TX FIFO topped up with counter words
static void send_task(void) {
  if (!tud_cdc_connected()) {
    return;
  }

#if SENDER_BYTES_PER_SEC
  uint64_t const allowed =
      (time_us_64() - session_start_us) * SENDER_BYTES_PER_SEC / 1000000;
  // At most 10 ms worth of credit, so an idle spell does not end in a burst
  uint64_t const max_credit = SENDER_BYTES_PER_SEC / 100 + sizeof(uint32_t) * CHUNK_WORDS;
  if (allowed > sent + max_credit) {
    sent = allowed - max_credit;
  }
#endif

  uint32_t chunk[CHUNK_WORDS];
  while (tud_cdc_write_available() >= sizeof(chunk)) {
#if SENDER_BYTES_PER_SEC
    if (allowed < sent + sizeof(chunk)) {
      break;
    }
    sent += sizeof(chunk);
#endif
    for (int i = 0; i < CHUNK_WORDS; i++) {
      chunk[i] = counter++; // RP2040/RP2350 are little-endian
    }
    tud_cdc_write(chunk, sizeof(chunk));
  }
  tud_cdc_write_flush();
}

static void led_task(void) {
#ifdef PICO_DEFAULT_LED_PIN
  if (tud_cdc_connected()) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
  } else {
    gpio_put(PICO_DEFAULT_LED_PIN, (to_ms_since_boot(get_absolute_time()) / 250) & 1);
  }
#endif
}

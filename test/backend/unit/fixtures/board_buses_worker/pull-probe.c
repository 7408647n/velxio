/*
 * pull-probe.c: an open-drain line for the QEMU worker's pad model
 * (test_board_buses_worker_pads.py).
 *
 *   LINE  the open-drain line: an input the chip watches on both edges,
 *         logging "line=<level>" for each, and pulls LOW while CTL is HIGH
 *         (vx_pin_set_mode(VX_OUTPUT_LOW)), releasing it (VX_INPUT) when CTL
 *         goes LOW again. A TM1637 ACK, a DS18B20 presence pulse and an I2C
 *         target's low are this shape.
 *   CTL   a GPIO the test drives from the guest to make the chip pull.
 *
 * On a board the line reads HIGH while nobody pulls it only through a
 * module's resistor, which the worker learns from the tab's bus map.
 */
#include "velxio-chip.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct {
  vx_pin line, ctl;
} probe_t;

static void on_line(void* ud, vx_pin pin, int value) {
  (void)ud;
  (void)pin;
  printf("line=%d\n", value);
}

static void on_ctl(void* ud, vx_pin pin, int value) {
  probe_t* s = (probe_t*)ud;
  (void)pin;
  vx_pin_set_mode(s->line, value ? VX_OUTPUT_LOW : VX_INPUT);
  printf("ctl=%d read=%d\n", value, vx_pin_read(s->line));
}

void chip_setup(void) {
  probe_t* s = (probe_t*)calloc(1, sizeof(probe_t));
  s->line = vx_pin_register("LINE", VX_INPUT);
  s->ctl = vx_pin_register("CTL", VX_INPUT);
  vx_pin_watch(s->line, VX_EDGE_BOTH, on_line, s);
  vx_pin_watch(s->ctl, VX_EDGE_BOTH, on_ctl, s);
  vx_log("pull probe ready");
}

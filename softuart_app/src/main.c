#include "fw/src/mgos.h"
#include "softuart.h"
#include "fw/src/mgos_timers.h"
#include "mgos_wifi.h"
#include "mgos_config.h"

double counter=0;

static void timer1_cb(void * arg) {
  unsigned char msg[3];
  msg[0] = 0xFE;
  msg[1] = 0x83;
  msg[2] = 0x83;
  mgos_softuart_send(msg, 3);
  (void) arg;
}

static void rx_cb(volatile char* stuff, unsigned int len) {
  LOG(LL_INFO, ("len: %d", len));
  for (int j=0; j<(int)len; j++) {
    LOG(LL_INFO, ("stuff: %02x", stuff[j]));
  }
  (void) stuff;
}

enum mgos_app_init_result mgos_app_init(void) {

  mgos_set_timer(50, 1, timer1_cb, NULL);

  mgos_softuart_add_cb(3, rx_cb);

  mgos_wifi_connect();

  return MGOS_APP_INIT_SUCCESS;
}
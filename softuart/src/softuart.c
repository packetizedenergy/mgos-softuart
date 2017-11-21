#include "softuart.h"
#include "mgos_bitbang.h"
#include "mgos_config_util.h"
#include "mgos_timers.h"
#include "mgos_hw_timers_hal.h"

// #define SOFTUART_DEBUGGING

/*
NOTES:
this only supports 8N1, LSB first
TRANSMIT: up to 115200 baud works fine
RECEIVE: not sure on limits yet, Mello meter optos only support up to 2400 baud
BYTES: 6 receive and 6 transmit, MAXIMUM (this is not a robust library)
*/

#define RX_BIT_TRANSITIONS_PER_BYTE 8
#define BIT_TRANSITIONS_PER_BYTE 7
#define RX_BYTE_TIMEOUT_MS 10

static volatile enum BIT_STATE {
  START_BIT = 0,
  DATA_BIT = 1,
  STOP_BIT = 2,
  IDLE_BIT = 3
} byte_state = IDLE_BIT;

static int tx_gpio = 14;
static int rx_gpio = 12;
// static volatile uint16_t baudrate = 2400;
static bool inverted_tx = 1;
static bool inverted_rx = 1;
static uint16_t bit_width_us = 417;
static uint16_t half_bitwidth_us = 208;
// static volatile uint16_t tx_spacing_us = 0;
static volatile uint8_t tx_bit_index = 0;
static volatile uint8_t tx_byte_index = 0;
static volatile uint8_t tx_byte_count = 0;
static volatile uint8_t rx_bit_index = 0;
// static volatile uint8_t rx_byte_index = 0;
static volatile uint8_t rx_byte_count = 0;
static volatile uint8_t rx_data = 0;
static volatile bool rx_framing_error = 0;

static void (*cb)(volatile char*, unsigned int);
static unsigned int num_bytes_cb;

static volatile int tx_timer_id;
static volatile int rx_timer_id;
static volatile int rx_start_timer_id;

static unsigned char tx_data[6]; // this is bad, max 6 bytes
static volatile char rx_buf[6]; // this is bad, max 6 bytes

// void clear_all_timers(void) {
//   mgos_clear_timer(tx_timer_id);
//   mgos_clear_timer(rx_timer_id);
//   mgos_clear_timer(rx_start_timer_id);
// }

void callback_caller(void *args) {
  LOG(LL_INFO, ("callback"));
  mgos_clear_timer(1);
  // cb(rx_buf, num_bytes_cb);
  rx_byte_count = 0;
  (void) args;
}

IRAM void send_bit_cb(void *args) { // this is a real HW ISR, so have to be fast
  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_write(15,1); 
  #endif
  switch (byte_state) {
    case START_BIT:
      mgos_gpio_write(tx_gpio, 0^inverted_tx); // send the start bit
      byte_state = DATA_BIT; // move to next bit
      break;
    case DATA_BIT:
      mgos_gpio_write(tx_gpio, (bool)(tx_data[tx_byte_index] & (1 << tx_bit_index))^inverted_tx ); // send the data bit
      if (tx_bit_index == BIT_TRANSITIONS_PER_BYTE) { // have finished this byte
        tx_bit_index = 0; // reset for next byte
        byte_state = STOP_BIT; // move on to stop bit
      } else { // still more bits in this byte
        tx_bit_index++;; // go to next data bit
      }
      break;
    case STOP_BIT:
      mgos_gpio_write(tx_gpio, 1^inverted_tx); // send the stop bit
      if (tx_byte_index == tx_byte_count) { // have finished the last byte
        mgos_clear_timer(1);
      } else { // haven't finished
        tx_byte_index++; // get ready for the next byte
        byte_state = IDLE_BIT;
      }
      break;
    case IDLE_BIT:
      mgos_gpio_write(tx_gpio, 1^inverted_tx); // send the idle bit
      byte_state = START_BIT;
      break;
  }
  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_write(15,0); 
  #endif
  (void) args;
}

IRAM void get_bit_cb(void *args) { // this is a real HW ISR, so have to be fast
  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_write(15,1); 
  #endif
  if (rx_bit_index < RX_BIT_TRANSITIONS_PER_BYTE) {
    if (!rx_bit_index) { // if we just got here
      mgos_clear_timer(1);
      rx_timer_id = mgos_set_hw_timer(bit_width_us, MGOS_TIMER_REPEAT | MGOS_ESP8266_HW_TIMER_NMI, (void (*)(void*))get_bit_cb, NULL); // keep getting bits
    }
    rx_data |= (mgos_gpio_read(rx_gpio)^inverted_rx)<<rx_bit_index; // grab the bit
  } else { // last bit
    rx_framing_error = ~(mgos_gpio_read(rx_gpio)^inverted_rx); // check stop bit for framing error
    mgos_clear_timer(1);
    mgos_gpio_enable_int(rx_gpio); // get looking for next start bit
    rx_buf[rx_byte_count] = rx_data;
    // memcpy(&rx_buf[rx_byte_count], &rx_data, 1); // this line causes firmware-main to crash very quickly
    if(rx_byte_count == num_bytes_cb-1 && cb != NULL) {
      mgos_invoke_cb((void (*)(void*))callback_caller, NULL, true);
    }
    rx_byte_count++;
  }
  rx_bit_index++; // get ready for the next bit
  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_write(15,0); 
  #endif
  (void) args;
}

IRAM void get_rx_start_bit_cb(int pin, void *args) { // this is a real HW ISR, so have to be fast
  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_write(15,1); 
  #endif
  if ((~mgos_gpio_read(rx_gpio))^inverted_rx) { // this is the start bit
    rx_data = 0; // reset data byte
    mgos_gpio_disable_int(rx_gpio);
    mgos_clear_timer(1);
    rx_start_timer_id = mgos_set_hw_timer(bit_width_us + half_bitwidth_us, MGOS_ESP8266_HW_TIMER_NMI, (void (*)(void*))get_bit_cb, NULL); // get lined up for the first data bit
    rx_bit_index = 0; // start over for new byte
  }
  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_write(15,0); 
  #endif
  (void) pin;
  (void) args;
}

void mgos_softuart_send(unsigned char *data, int len) {
  tx_byte_index = 0; // reset byte count
  tx_byte_count = len-1; // get number of bytes
  tx_bit_index = 0; // start at the first bit
  byte_state = START_BIT;

  memcpy(tx_data, data, len); // otherwise it's gone by the time we get to the ISR

  mgos_clear_timer(1);
  tx_timer_id = mgos_set_hw_timer(bit_width_us, MGOS_TIMER_REPEAT | MGOS_ESP8266_HW_TIMER_NMI, (void (*)(void*))send_bit_cb, NULL);
}

bool mgos_softuart_init(void) {
  mgos_gpio_set_mode(tx_gpio, MGOS_GPIO_MODE_OUTPUT); // set output
  mgos_gpio_set_mode(rx_gpio, MGOS_GPIO_MODE_INPUT); // set input
  mgos_gpio_write(tx_gpio, 1^inverted_tx); // get the pin to idle state

  #ifdef SOFTUART_DEBUGGING 
  mgos_gpio_set_mode(15, MGOS_GPIO_MODE_OUTPUT);
  mgos_gpio_write(15, 1);
  #endif

  mgos_gpio_set_int_handler_isr(rx_gpio, inverted_rx?MGOS_GPIO_INT_EDGE_POS:MGOS_GPIO_INT_EDGE_NEG,(mgos_gpio_int_handler_f)get_rx_start_bit_cb, NULL);
  mgos_gpio_enable_int(rx_gpio);

  mgos_clear_timer(1);

  return true;
}

void mgos_softuart_add_cb(unsigned int num_bytes, void (*callback)(volatile char*, unsigned int)) {
  cb = callback;
  num_bytes_cb = num_bytes;
}
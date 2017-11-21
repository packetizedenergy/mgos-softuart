#ifndef SOFTUART_H
#define SOFTUART_H

#include <stdio.h>
#include "softuart.h"

#define LIBNAME mgos_softuart
#define LIBNAME_init mgos_softuart_init

typedef unsigned char byte;

#include "mgos_gpio.h"
#include "mgos_hal.h"
#include "mgos_timers.h"
#include "mgos_config.h"
#include "mgos.h"
#include "common/mbuf.h"

enum softuart_status {
    SOFTUART_AVAILABLE,
    SOFTUART_EMPTY,
    SOFTUART_FRAMING_ERR,
};

struct mgos_softuart;

#ifdef __cplusplus
extern "C" {
#endif

void mgos_softuart_send(unsigned char *data, int len);

int mgos_softuart_available(void);

char *mgos_softuart_readbuf(void);

void mgos_softuart_add_cb(unsigned int num_bytes, void (*cb)(volatile char*, unsigned int));

bool mgos_softuart_init(void);


#ifdef __cplusplus
}
#endif

#endif
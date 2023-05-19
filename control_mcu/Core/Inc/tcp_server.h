#ifndef INC_TCP_SERVER_H_
#define INC_TCP_SERVER_H_

#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include <string.h>

typedef struct {
    int x_en;
    int y_en;
    float x_position;
    float y_position;
    float x_velocity;
    float y_velocity;
} config_packet_struct;

enum tcp_server_states
{
  ES_NONE = 0,
  ES_ACCEPTED,
  ES_RECEIVED,
  ES_CLOSING
};

struct tcp_server_state_struct
{
  uint8_t           state   ;
  uint8_t           retries ;
  struct  tcp_pcb   *pcb    ;
  struct  pbuf      *p      ;
};

err_t init_tcp_server(void);

#endif

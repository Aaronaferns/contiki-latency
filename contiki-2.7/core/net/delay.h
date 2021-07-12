

#ifndef DELAY_H
#define DELAY_H

#include <stdio.h>
#include "net/packetbuf.h"
#include "net/queuebuf.h"

//void time_memb_init();
//void neighbor_info_send_mac (rimeaddr_t *dest) ;

#define MAX_DELAY 3000
typedef uint32_t delay_t;

/* Structure used to record times for delay computing */
struct time_queue {
	struct time_queue *next;
	rimeaddr_t dest;
	uint32_t before_mac;
};

#define MAX_QUEUED_PACKETS QUEUEBUF_NUM

uint32_t before_trans;
uint32_t after_ack;

#endif /* DELAY_H */

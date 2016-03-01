/*
 * Copyright (c) 2011, Swedish Institute of Computer Science
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */
/*
 * This code is almost device independent and should be easy to port.
 */

#include "contiki.h"

#include "dev/spi.h"
#include "rf.h"

#include "net/packetbuf.h"
#include "net/rime/rimestats.h"
#include "net/netstack.h"

#include "sys/timetable.h"
#include <string.h>

#define UZ2400_MAX_PACKET_LEN 127


#ifndef UZ2400_CONF_AUTOACK
#define UZ2400_CONF_AUTOACK 0
#endif /* UZ2400_CONF_AUTOACK */

#define WITH_SEND_CCA 1

#define FOOTER_LEN 2

#define FOOTER1_CRC_OK      0x80
#define FOOTER1_CORRELATION 0x7f

#include <stdio.h>
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif

#define CHECKSUM_LEN 2
#define AUX_LEN (CHECKSUM_LEN + FOOTER_LEN)

#if 0 && DEBUG
#include "dev/leds.h"
#define LEDS_ON(x) leds_on(x)
#define LEDS_OFF(x) leds_off(x)
#else
#define LEDS_ON(x)
#define LEDS_OFF(x)
#endif

void uz2400_arch_init(void);

/* XXX hack: these will be made as Chameleon packet attributes */
rtimer_clock_t uz2400_time_of_arrival, uz2400_time_of_departure;

int uz2400_authority_level_of_sender;

int uz2400_packets_seen, uz2400_packets_read;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

volatile uint8_t uz2400_sfd_counter;
volatile uint16_t uz2400_sfd_start_time;
volatile uint16_t uz2400_sfd_end_time;

static volatile uint16_t last_packet_timestamp;
/*---------------------------------------------------------------------------*/
PROCESS(uz2400_process, "UZ2400 driver");
/*---------------------------------------------------------------------------*/


int uz2400_on(void);
int uz2400_off(void);

int uz2400_init(void);
static int uz2400_read(void *buf, unsigned short bufsize);

static int uz2400_prepare(const void *data, unsigned short len);
static int uz2400_transmit(unsigned short len);
static int uz2400_send(const void *data, unsigned short len);

static int uz2400_receiving_packet(void);
static int pending_packet(void);
static int uz2400_cca(void);

int uz2400_interrupt(void);

/* static int detected_energy(void); */

signed char uz2400_last_rssi;
uint8_t uz2400_last_correlation;

const struct radio_driver uz2400_driver =
  {
    uz2400_init,
    uz2400_prepare,
    uz2400_transmit,
    uz2400_send,
    uz2400_read,
    /* uz2400_set_channel, */
    /* detected_energy, */
    uz2400_cca,
    uz2400_receiving_packet,
    pending_packet,
    uz2400_on,
    uz2400_off,
  };

static uint8_t receive_on;

static int channel;

/*---------------------------------------------------------------------------*/
static uint8_t locked, lock_on, lock_off;

static void
on(void)
{
	uz2400_on();
	receive_on = 1;
}
static void
off(void)
{
	uz2400_off();
	receive_on = 0;
}
/*---------------------------------------------------------------------------*/
static void
set_txpower(uint8_t power)
{
}
/*---------------------------------------------------------------------------*/
int
uz2400_init(void)
{
  uz_init();
  uz_rx_normal_mode();
  uz_set_tx_power(0);
  set_interrupt_function(uz2400_interrupt);
  process_start(&uz2400_process, NULL);
  return 1;
}
/*---------------------------------------------------------------------------*/
static int
uz2400_transmit(unsigned short payload_len)
{
  return uz_transmit();
}
/*---------------------------------------------------------------------------*/
static int
uz2400_prepare(const void *payload, unsigned short payload_len)
{
  return uz_prepare((UINT8 *) payload, (UINT8) payload_len);
}
/*---------------------------------------------------------------------------*/
static int
uz2400_send(const void *payload, unsigned short payload_len)
{
  //int ret = 0;
  //halRfDisableRxInterrupt();
  
  /*printf("Test: %d\n", payload_len);
  printf("================================\n");
  printf("Sending\n");
  printf("================================\n");
  int i;
  for(i = 0; i < payload_len; i++) printf("%2x ", *(UINT8 *)(payload+i));
  printf("\n================================\n");*/
  
  uz2400_prepare(payload, payload_len);
  return uz2400_transmit(payload_len);
  //ret = uz2400_transmit(payload_len);
  //halRfEnableRxInterrupt();
  //return ret;
}
/*---------------------------------------------------------------------------*/
int
uz2400_off(void)
{
  PRINTF("uz2400.c: Turning uz2400 off.\n");
  return 1;
}
/*---------------------------------------------------------------------------*/
int
uz2400_on(void)
{
  PRINTF("uz2400.c: Turning uz2400 on.\n");
  return 1;
}
/*---------------------------------------------------------------------------*/
int
uz2400_get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
int
uz2400_set_channel(int c)
{
  uz_set_channel(c);
}
/*---------------------------------------------------------------------------*/
void
uz2400_set_pan_addr(unsigned pan,
                    unsigned addr,
                    const uint8_t *ieee_addr)
{
    uz_set_panId(pan);
    uz_set_nwk_addr(addr);
    uz_set_mac_address(ieee_addr);
}
/*---------------------------------------------------------------------------*/
/*
 * Interrupt leaves frame intact in FIFO.
 */
#if UZ2400_TIMETABLE_PROFILING
#define uz2400_timetable_size 16
TIMETABLE(uz2400_timetable);
TIMETABLE_AGGREGATE(aggregate_time, 10);
#endif /* UZ2400_TIMETABLE_PROFILING */
int
uz2400_interrupt(void)
{
  //UZ2400_CLEAR_FIFOP_INT();
  //printf("UZ2400.c: Handle interrupt\n");
  process_poll(&uz2400_process);

    //int len;
    //packetbuf_clear();
    //packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    //len = uz2400_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    //printf("len: %d\n", len);
    //packetbuf_set_datalen(len);

    //NETSTACK_RDC.input();
//#if UZ2400_TIMETABLE_PROFILING
//  timetable_clear(&uz2400_timetable);
//  TIMETABLE_TIMESTAMP(uz2400_timetable, "interrupt");
//#endif /* UZ2400_TIMETABLE_PROFILING */

  last_packet_timestamp = uz2400_sfd_start_time;
  uz2400_packets_seen++;
  return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uz2400_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  PRINTF("uz2400_process: started\n");

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
//#if UZ2400_TIMETABLE_PROFILING
//    TIMETABLE_TIMESTAMP(uz2400_timetable, "poll");
//#endif /* UZ2400_TIMETABLE_PROFILING */

    PRINTF("uz2400_process: calling receiver callback\n");

    packetbuf_clear();
    packetbuf_set_attr(PACKETBUF_ATTR_TIMESTAMP, last_packet_timestamp);
    
    len = uz2400_read(packetbuf_dataptr(), PACKETBUF_SIZE);
    packetbuf_set_datalen(len);

    NETSTACK_RDC.input();
    /* flushrx(); */
//#if UZ2400_TIMETABLE_PROFILING
//    TIMETABLE_TIMESTAMP(uz2400_timetable, "end");
//    timetable_aggregate_compute_detailed(&aggregate_time,
//                                         &uz2400_timetable);
//    timetable_clear(&uz2400_timetable);
//#endif /* UZ2400_TIMETABLE_PROFILING */
	//EXTI4_ClearBit();
  }

  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
static int
uz2400_read(void *buf, unsigned short bufsize)
{
  uint8_t len;
  
  len = uz_rx(buf);
  
  /*printf("================================\n");
  printf("Reading\n");
  printf("================================\n");
  int i;
  for(i = 0; i < len; i++) printf("%2x ", *(UINT8 *)(buf+i));
  printf("\n================================\n");*/
  
  if(len > bufsize) {
	  uz_rx_flush();
	  return 0;
  }
  
  if(len > UZ2400_MAX_PACKET_LEN) {
    /* Oops, we must be out of sync. */
    return 0;
  }
  
  if(len <= AUX_LEN) {
    return 0;
  }

  if(len - AUX_LEN > bufsize) {
    return 0;
  }
  
  if(len < AUX_LEN) {
    return 0;
  }

  // still bug here, check the datasheet
  uz2400_last_rssi = buf+(len-1);
  uz2400_last_correlation = buf+(len-2); // LQI
  
  if(len < AUX_LEN) {
    return 0;
  }

  return len - AUX_LEN;
  
  //if(len > 0) process_poll(&uz2400_process);
  //printf("uz2400_read length: %d\n", len);
  return len;
}
/*---------------------------------------------------------------------------*/
void
uz2400_set_txpower(uint8_t power)
{
  set_txpower(power);
}
/*---------------------------------------------------------------------------*/
int
uz2400_get_txpower(void)
{
  uint8_t power;
  return power;
}
/*---------------------------------------------------------------------------*/
int
uz2400_rssi(void)
{
  int rssi;
  return rssi;
}
/*---------------------------------------------------------------------------*/
/*
static int
detected_energy(void)
{
  return uz2400_rssi();
}
*/
/*---------------------------------------------------------------------------*/
int
uz2400_cca_valid(void)
{
}
/*---------------------------------------------------------------------------*/
int
uz2400_cca(void)
{
	//return halRfcca();
	return 0;
}
/*---------------------------------------------------------------------------*/
int
uz2400_receiving_packet(void)
{
	//return halRfReceiving();
	return 0;
}
/*---------------------------------------------------------------------------*/
static int
pending_packet(void)
{
  //return halRfPending();
  return 0;
}
/*---------------------------------------------------------------------------*/
void
uz2400_set_cca_threshold(int value)
{
}
/*---------------------------------------------------------------------------*/

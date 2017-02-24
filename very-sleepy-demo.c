/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/stimer.h"
#include "sys/process.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "net/netstack.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "rest-engine.h"
#include "er-coap.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
/*---------Librerías y definiciones para Multicast---------------------------*/
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ipv6/multicast/uip-mcast6.h"

#include <string.h>

#define MCAST_SINK_UDP_PORT 5386 /*Host byte order*/
static struct uip_udp_conn *sink_conn;
#define UIP_IP_BUF  ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])
static uip_ds6_maddr_t * group_dir;
/*---------------------------------------------------------------------------*/
/* Normal mode duration params in seconds */
#define NORMAL_OP_DURATION_DEFAULT 10
#define NORMAL_OP_DURATION_MIN     10
#define NORMAL_OP_DURATION_MAX     60
/*---------------------------------------------------------------------------*/
/* Observer notification period params in seconds */
#define PERIODIC_INTERVAL_DEFAULT  30
#define PERIODIC_INTERVAL_MIN      30
#define PERIODIC_INTERVAL_MAX      86400 /* 1 day */
/*---------------------------------------------------------------------------*/
#define VERY_SLEEPY_MODE_OFF 0
#define VERY_SLEEPY_MODE_ON  1
/*---------------------------------------------------------------------------*/
#define MAC_CAN_BE_TURNED_OFF  0
#define MAC_MUST_STAY_ON       1

#define KEEP_MAC_ON_MIN_PERIOD 10 /* secs */
/*---------------------------------------------------------------------------*/
#define PERIODIC_INTERVAL         CLOCK_SECOND
/*----------------------------POST_STATUS (SOLO Potencias de 2!!!!!)-----------------------------------------------*/
/* Son macros para determinar si ha habido una modificación de los parámetros o algún error*/
#define POST_STATUS_BAD           0x80
#define POST_STATUS_HAS_MODE      0x40
#define POST_STATUS_HAS_DURATION  0x20
#define POST_STATUS_HAS_INTERVAL  0x10
#define POST_STATUS_NONE          0x00      //Máscara inicial
#define POST_STATUS_HAS_CODE      0x01
#define POST_STATUS_HAS_MASK      0x02
#define POST_STATUS_HAS_JOINED    0x04
/*---------------------------Estructura de configuracion------------------------------------------------*/
typedef struct sleepy_config_s {
  unsigned long interval;
  unsigned long duration;
  uint8_t mode;
} sleepy_config_t;

sleepy_config_t config;
/*---------------------------Estructura de Operacion------------------------------------------------*/
#define OPERATION_DEFAULT 1               //Operacion por defecto (1)
#define OPERATION_MIN     1               //Código Minimo de operacion
#define OPERATION_MAX     255             //Código máximo de operacion
#define MASK_DEFAULT      0b00000001      //Máscara de operacion por defecto
#define MASK_MIN          0b00000001      //Máscara de operacion mínima
#define MASK_MAX          0b11111111      //Máscara de operacion máxima

typedef struct operation_s {
  uint8_t code;
  uint8_t mask;
} operation_t;

operation_t operation;
/*-----------------------ESTADOS----------------------------------------------*/
#define STATE_NORMAL           0
#define STATE_NOTIFY_OBSERVERS 1
#define STATE_VERY_SLEEPY      2
/*----------------------------------------------------------------------------*/
static struct stimer st_duration;               //Timer de duración
static struct stimer st_interval;               //Timer de intervalo
static struct stimer st_min_mac_on_duration;    //Timer de mínima duración MAC encendida
static struct etimer et_periodic;               //Timer de interrupción periodica (1s)
static process_event_t event_new_config;        //Evento de Configuracion
static process_event_t event_new_op;            //Evento de Operacion
static process_event_t event_join_mcast;		//Evento de union a grupo multicast.
static uint8_t state;                           //Variable de estado
/*---------------------------------------------------------------------------*/
const char *not_supported_msg = "Supported:text/plain,application/json";
/*------------------------Función de unión a multicast group-----------------*/
static uip_ds6_maddr_t * join_mcast_group(void)
{
  uip_ipaddr_t addr;
  uip_ds6_maddr_t *rv;

  /* First, set our v6 global (DUDA DE SI ES NECESARIO O NO)*/
  //suip_ip6addr(&addr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 0);    //Crea una direccion ipv6 ~(fd00::)
  //uip_ds6_set_addr_iid(&addr, &uip_lladdr);                           //Fija los ultimos 64 bits de la dirección con la dirección MAC
  //uip_ds6_addr_add(&addr, 0, ADDR_AUTOCONF);                          //Añade una dirección unicast (addr) a la interfaz

  /*
   * IPHC will use stateless multicast compression for this destination
   * (M=1, DAC=0), with 32 inline bits (1E 89 AB CD)
   */
  uip_ip6addr(&addr, 0xBBBB,0,0,0,0,0,0x0089,0xABCD);                   //Crea la direccion (BBBB::89:ABCD)
  rv = uip_ds6_maddr_add(&addr);                                      //Añade la dirección multicast rv a la interfaz (Sup)

  return rv;
}
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS(very_sleepy_demo_process, "CC13xx/CC26xx very sleepy process");
AUTOSTART_PROCESSES(&very_sleepy_demo_process);
/*---------------------------------------------------------------------------*/
//Peticion de valores de Sensores (GET)
static void readings_get_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  unsigned int accept = -1;
  int voltage;			//Voltaje
  int tempbat;			//Temperatura de la batería

  if(request != NULL) {							//Si se acepta la petición
    REST.get_header_accept(request, &accept);
  }
  tempbat = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);	              //Obtiene el valor de la temperatura de la batería
  voltage = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);	              //Obtiene el valor del voltaje
  if(accept == -1 || accept == REST.type.APPLICATION_JSON) {
    REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,"{\"Tempbat\":{\"v\":%d,\"u\":\"C\"},\"Voltage\":{\"v\":%d,\"u\":\"mV\"}}",tempbat, (voltage * 125) >> 5);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else if(accept == REST.type.TEXT_PLAIN) {
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "Tempbat=%dC, Voltage=%dmV",tempbat, (voltage * 125) >> 5);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else {
    REST.set_response_status(response, REST.status.NOT_ACCEPTABLE);
    REST.set_response_payload(response, not_supported_msg,strlen(not_supported_msg));
  }
}
/*---------------------------------------------------------------------------*/
RESOURCE(readings_resource, "title=\"Sensor Readings\";obs",readings_get_handler, NULL, NULL, NULL);  //Recurso lectura de Sensores
/*---------------------------------------------------------------------------*/
static void groupipv6_get_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  unsigned int accept = -1;


  if(request != NULL) {							//Si se acepta la petición
    REST.get_header_accept(request, &accept);
  }

  if(accept == -1 || accept == REST.type.APPLICATION_JSON) {
    REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,"{\"Group_Dir\":{[%x:%x:%x:%x:%x:%x:%x:%x]}}",group_dir->ipaddr.u16[0],group_dir->ipaddr.u16[1],group_dir->ipaddr.u16[2],
    group_dir->ipaddr.u16[3],group_dir->ipaddr.u16[4],group_dir->ipaddr.u16[5],group_dir->ipaddr.u16[6],group_dir->ipaddr.u16[7]); //Posible conflicto de tipo
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else if(accept == REST.type.TEXT_PLAIN) {
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "Group_Dir=[%x:%x:%x:%x:%x:%x:%x:%x]",group_dir->ipaddr.u16[0],group_dir->ipaddr.u16[1],group_dir->ipaddr.u16[2],
    group_dir->ipaddr.u16[3],group_dir->ipaddr.u16[4],group_dir->ipaddr.u16[5],group_dir->ipaddr.u16[6],group_dir->ipaddr.u16[7]);        //Posible conflicto de tipo
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else {
    REST.set_response_status(response, REST.status.NOT_ACCEPTABLE);
    REST.set_response_payload(response, not_supported_msg,strlen(not_supported_msg));
  }
}
static void groupipv6_post_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  const char *ptr = NULL;
  char tmp_buf[16];
  uint8_t unido = 0;
  uint8_t post_status = POST_STATUS_NONE;
  int rv;

  rv = REST.get_post_variable(request, "Unido", &ptr); //Obtiene la variable "unido"
  if(rv && rv < 16) {
    memset(tmp_buf, 0, sizeof(tmp_buf));  //Reserva espacio para tmp_buf
    memcpy(tmp_buf, ptr, rv);             //Pasa el valor de la direccion ptr a tmp_buf
    rv = atoi(tmp_buf);                   //Convierte un String a Entero

    unido = rv;
    if(unido==1) {                           //Si unido==1
      post_status |= POST_STATUS_HAS_JOINED;
      group_dir = join_mcast_group();                           //Unión al grupo multicast
      sink_conn = udp_new(NULL, UIP_HTONS(0), NULL);            //Nueva conexión UDP (con puerto por defecto)
      udp_bind(sink_conn, UIP_HTONS(MCAST_SINK_UDP_PORT));      //Fija el puerto para la conexión UDP
    }
    else if (unido == 0){
            //TODO:Introducir código para abandonar el grupo
    } 
    else {                             
      post_status |= POST_STATUS_BAD;//Se suma la máscara a post_status
    }
  }
  if((post_status & POST_STATUS_BAD) == POST_STATUS_BAD ||
     post_status == POST_STATUS_NONE) {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,
             "Unido=0|1");
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
    return;
  }


  process_post(&very_sleepy_demo_process, event_join_mcast, NULL);
}
/*---------------------------------------------------------------------------*/
RESOURCE(groupipv6, "title=\"Group Direction: ""GET|POST Unido=0|1\";rt=\"Control\"",groupipv6_get_handler, groupipv6_post_handler, NULL, NULL);  //

/*---------------------------------------------------------------------------*/
static void conf_get_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  unsigned int accept = -1;

  if(request != NULL) {
    REST.get_header_accept(request, &accept);
  }

  if(accept == -1 || accept == REST.type.APPLICATION_JSON) {
    REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "{\"config\":{\"mode\":%u,\"duration\":%lu,\"interval\":%lu}}",config.mode, config.duration, config.interval);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else if(accept == REST.type.TEXT_PLAIN) {
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,"Mode=%u, Duration=%lusecs, Interval=%lusecs",config.mode, config.duration, config.interval);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else {
    REST.set_response_status(response, REST.status.NOT_ACCEPTABLE);
    REST.set_response_payload(response, not_supported_msg,
                              strlen(not_supported_msg));
  }
}
/*---------------------------------------------------------------------------*/
static void conf_post_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  const char *ptr = NULL;
  char tmp_buf[16];
  unsigned long interval = 0;
  unsigned long duration = 0;
  uint8_t mode = VERY_SLEEPY_MODE_OFF;
  uint8_t post_status = POST_STATUS_NONE;
  int rv;

  rv = REST.get_post_variable(request, "mode", &ptr);     //Obtiene la variable "mode"
  if(rv && rv < 16) {
    memset(tmp_buf, 0, sizeof(tmp_buf));
    memcpy(tmp_buf, ptr, rv);
    rv = atoi(tmp_buf);

    if(rv == 1) {
      mode = VERY_SLEEPY_MODE_ON;
      post_status |= POST_STATUS_HAS_MODE;
    } else if(rv == 0) {
      mode = VERY_SLEEPY_MODE_OFF;
      post_status |= POST_STATUS_HAS_MODE;
    } else {
      post_status = POST_STATUS_BAD;
    }
  }

  rv = REST.get_post_variable(request, "duration", &ptr); //Obtiene la variable "duration"
  if(rv && rv < 16) {
    memset(tmp_buf, 0, sizeof(tmp_buf));
    memcpy(tmp_buf, ptr, rv);
    rv = atoi(tmp_buf);

    duration = (unsigned long)rv;
    if(duration < NORMAL_OP_DURATION_MIN || duration > NORMAL_OP_DURATION_MAX) {
      post_status = POST_STATUS_BAD;
    } else {
      post_status |= POST_STATUS_HAS_DURATION;
    }
  }

  rv = REST.get_post_variable(request, "interval", &ptr); //Obtiene la variable "Interval"
  if(rv && rv < 16) {
    memset(tmp_buf, 0, sizeof(tmp_buf));
    memcpy(tmp_buf, ptr, rv);
    rv = atoi(tmp_buf);
    interval = (unsigned long)rv;
    if(interval < PERIODIC_INTERVAL_MIN || interval > PERIODIC_INTERVAL_MAX) {
      post_status = POST_STATUS_BAD;
    } else {
      post_status |= POST_STATUS_HAS_INTERVAL;
    }
  }

  if((post_status & POST_STATUS_BAD) == POST_STATUS_BAD ||
     post_status == POST_STATUS_NONE) {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,
             "mode=0|1&duration=[%u,%u]&interval=[%u,%u]",
             NORMAL_OP_DURATION_MIN, NORMAL_OP_DURATION_MAX,
             PERIODIC_INTERVAL_MIN, PERIODIC_INTERVAL_MAX);

    REST.set_response_payload(response, buffer, strlen((char *)buffer));
    return;
  }

  /* Values are sane. Update the config and notify the process */
  if(post_status & POST_STATUS_HAS_MODE) {
    config.mode = mode;
  }

  if(post_status & POST_STATUS_HAS_INTERVAL) {
    config.interval = interval;
  }

  if(post_status & POST_STATUS_HAS_DURATION) {
    config.duration = duration;
  }

  process_post(&very_sleepy_demo_process, event_new_config, NULL);
}
/*------------Recurso de configuracion modo sueño profundo---------------------------------------------------------------*/
RESOURCE(very_sleepy_conf,"title=\"Very sleepy conf: ""GET|POST mode=0|1&interval=<secs>&duration=<secs>\";rt=\"Control\"",conf_get_handler, conf_post_handler, NULL, NULL);
/*---------------------------------------------------------------------------*/
static void op_get_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  unsigned int accept = -1;

  if(request != NULL) {
    REST.get_header_accept(request, &accept);
  }

  if(accept == -1 || accept == REST.type.APPLICATION_JSON) {
    REST.set_header_content_type(response, REST.type.APPLICATION_JSON);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE, "{\"Operation\":{\"Code\":%u,\"Mask\":%u}}",operation.code,operation.mask);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else if(accept == REST.type.TEXT_PLAIN) {
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,"Code=%u, Mask=%u",operation.code,operation.mask);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
  } else {
    REST.set_response_status(response, REST.status.NOT_ACCEPTABLE);
    REST.set_response_payload(response, not_supported_msg,
                              strlen(not_supported_msg));
  }
}
/*---------------------------------------------------------------------------*/
static void op_post_handler(void *request, void *response, uint8_t *buffer,uint16_t preferred_size, int32_t *offset)
{
  const char *ptr = NULL;
  char tmp_buf[16];
  uint8_t code = 0;
  uint8_t mask = 0;
  uint8_t post_status = POST_STATUS_NONE;
  int rv;

  rv = REST.get_post_variable(request, "code", &ptr); //Obtiene la variable "code"
  if(rv && rv < 16) {
    memset(tmp_buf, 0, sizeof(tmp_buf));  //Reserva espacio para tmp_buf
    memcpy(tmp_buf, ptr, rv);             //Pasa el valor de la direccion ptr a tmp_buf
    rv = atoi(tmp_buf);                   //Convierte un String a Entero

    code = rv;
    if(!code) {                           //Si code==0
      post_status = POST_STATUS_BAD;      //Fallo en POST
    } else {                              //Code != 0
      post_status |= POST_STATUS_HAS_CODE;//Se suma la máscara a post_status
    }
  }

  rv = REST.get_post_variable(request, "mask", &ptr); //Obtiene la variable "mask"
  if(rv && rv < 16) {
    memset(tmp_buf, 0, sizeof(tmp_buf));
    memcpy(tmp_buf, ptr, rv);
    rv = atoi(tmp_buf);

    mask = rv;
    if(!mask) {
      post_status = POST_STATUS_BAD;
    } else {
      post_status |= POST_STATUS_HAS_MASK;
    }
  }
  if((post_status & POST_STATUS_BAD) == POST_STATUS_BAD ||
     post_status == POST_STATUS_NONE) {
    REST.set_response_status(response, REST.status.BAD_REQUEST);
    snprintf((char *)buffer, REST_MAX_CHUNK_SIZE,
             "Code=[%u,%u]&Mask=[%u,%u]",OPERATION_MIN,OPERATION_MAX,MASK_MIN,MASK_MAX);
    REST.set_response_payload(response, buffer, strlen((char *)buffer));
    return;
  }

  /* Values are sane. Update the config and notify the process */
  if(post_status & POST_STATUS_HAS_CODE) {
    operation.code = code;
  }
  if(post_status & POST_STATUS_HAS_MASK) {
    operation.mask = mask;
  }


  process_post(&very_sleepy_demo_process, event_new_op, NULL);
}
/*------------Recurso de configuracion operacion---------------------------------------------------------------*/
RESOURCE(operation_make,"title=\"Operation Make: ""GET|POST Code=<code>&Mask=<mask_code>\";rt=\"Control\"",op_get_handler, op_post_handler, NULL, NULL);
/*---------------------------------------------------------------------------*/
/*
 * If our preferred parent is not NBR_REACHABLE in the ND cache, NUD will send
 * a unicast NS and wait for NA. If NA fails then the neighbour will be removed
 * from the ND cache and the default route will be deleted. To prevent this,
 * keep the MAC on until the parent becomes NBR_REACHABLE. We also keep the MAC
 * on if we are about to do RPL probing.
 *
 * In all cases, the radio will be locked on for KEEP_MAC_ON_MIN_PERIOD secs
 */
static uint8_t keep_mac_on(void)
{
  uip_ds6_nbr_t *nbr;
  uint8_t rv = MAC_CAN_BE_TURNED_OFF;

  if(!stimer_expired(&st_min_mac_on_duration)) {      //Si el timer aún no ha cumplido
    return MAC_MUST_STAY_ON;                          //MAC activada
    printf("MAC activada\n");
  }

	#if RPL_WITH_PROBING
  		/* Determine if we are about to send a RPL probe */
  		if(CLOCK_LT(etimer_expiration_time(&rpl_get_default_instance()->probing_timer.etimer),(clock_time() + PERIODIC_INTERVAL))) {
    		rv = MAC_MUST_STAY_ON;
    		printf("MAC activada\n");

  		}
	#endif

  /* It's OK to pass a NULL pointer, the callee checks and returns NULL */
  nbr = uip_ds6_nbr_lookup(uip_ds6_defrt_choose());

  if(nbr == NULL) {
    /* We don't have a default route, or it's not reachable (NUD likely). */
    rv = MAC_MUST_STAY_ON;
    printf("MAC activada\n");

  } else {
    if(nbr->state != NBR_REACHABLE) {
      rv = MAC_MUST_STAY_ON;
      printf("MAC activada\n");


    }
  }

  if(rv == MAC_MUST_STAY_ON && stimer_expired(&st_min_mac_on_duration)) {     //Si MAC debe estar activada y el timer ha expirado
    stimer_set(&st_min_mac_on_duration, KEEP_MAC_ON_MIN_PERIOD);              //Se reinicia el timer
  }

  return rv;
}
/*---------------------------------------------------------------------------*/
static void switch_to_normal(void)
{
  state = STATE_NOTIFY_OBSERVERS;

  /*
   * Stay in normal mode for 'duration' secs.
   * Transition back to normal in 'interval' secs, _including_ 'duration'
   */
  stimer_set(&st_duration, config.duration);
  stimer_set(&st_interval, config.interval);
}
/*---------------------------------------------------------------------------*/
static void switch_to_very_sleepy(void)
{
  state = STATE_VERY_SLEEPY;
}

PROCESS_THREAD(very_sleepy_demo_process, ev, data)
{
	/*Cada segundo evalúa el estado del micro y dependiendo de lo que haya ocurrido continua durmiendo o despierta*/
  uint8_t mac_keep_on;

  PROCESS_BEGIN();

  SENSORS_ACTIVATE(batmon_sensor);			             		//Inicia el sensor de la batería
  config.mode = VERY_SLEEPY_MODE_OFF;					       //Configura el Modo por defecto
  config.interval = PERIODIC_INTERVAL_DEFAULT;			 		//Configura el Intervalo por defecto (30s)
  config.duration = NORMAL_OP_DURATION_DEFAULT;			 		//Configura la duracion por defecto  (10s)
  operation.code = OPERATION_DEFAULT;                			//Ninguna Operacion en marcha
  operation.mask= MASK_DEFAULT;                      			//Máscara para operación por defecto
  state = STATE_NORMAL;									        //Estado de inicio (nec al reset)

  event_new_config = process_alloc_event();				   		//Se reserva espacio para evento de nueva configuración
  event_new_op = process_alloc_event();              			//Se reserva espacio para evento de nueva operacion
  event_join_mcast = process_alloc_event();						//Se reserva espacio para evento mcast
  

  rest_init_engine();									               //Activa el motor REST (permite el recibir y mandar datos)

  readings_resource.flags += IS_OBSERVABLE;
  rest_activate_resource(&readings_resource, "sen/readings");       //Activa el recurso de lecturas
  rest_activate_resource(&very_sleepy_conf, "very_sleepy_config");  //Activa el recurso de configuracion
  rest_activate_resource(&operation_make, "Operation Make");        //Activa el recurso de Operacion
  rest_activate_resource(&groupipv6,"Grupo Multicast");
  
  printf("Very Sleepy Demo Process\n");

  switch_to_normal();									              //Pasa a normal (REACHABLE) Realmente pasa al modo NOTIFY_OBSERVERS (1)

  etimer_set(&et_periodic, PERIODIC_INTERVAL);			//Fija el timer de intervalo (no configurable) un segundo

  while(1) {

    PROCESS_YIELD();

    if(ev == sensors_event && data == &button_left_sensor) {	//Si se pulsa el boton se pasa a notificar observadores
      switch_to_normal();
      leds_off(LEDS_RED);         //Al pulsar operario se apaga el LED ROJO
      printf("Apagado LED Rojo (pulsado botón)\n");
    }
    if(ev == event_join_mcast){
    	printf("Evento mcast detectado\n");
    }
    if(ev == event_new_config) {								              //Si entra una nueva configuracion (¿igual que switch_to_normal?)
      stimer_set(&st_interval, config.interval);				      //Se fija el nuevo intervalo (No es de evento, hay que evaluarlo)
      stimer_set(&st_duration, config.duration);				      //Se fija la nueva duracion (No es de evento, hay que evaluarlo)
      printf("Detectado evento de nueva configuración, fijando timers de intervalo y duracion\n");
    }
    if(ev == event_new_op){
      if(operation.code & operation.mask){                    //Si la operacion corresponde con la máscara del dispositivo
        leds_on(LEDS_RED);                                    //Enciende el LED ROJO
        printf("Operación coincidente en máscara de dispositivo\n");
      }else{                                                  //En caso contrario
        leds_off(LEDS_RED);                                   //Apaga los leds (Medida para solventar error de operario)
      }
    }
    if((ev == PROCESS_EVENT_TIMER && data == &et_periodic) ||		   //Salta el timer de intervalo un segundo
       (ev == sensors_event && data == &button_left_sensor) ||		 //Se pulsa el botón
       (ev == event_new_config)|| (ev == event_new_op)) {					 //Entra una nueva configuracion o una operacion

      /*
       * Determine if the stack is about to do essential network maintenance
       * and, if so, keep the MAC layer on
       */
      mac_keep_on = keep_mac_on();				//Capa MAC activada

      if(mac_keep_on == MAC_MUST_STAY_ON || state != STATE_VERY_SLEEPY) {		//Si MAC activada y modo !Sleepy
        leds_on(LEDS_GREEN);                                                //Si el dispositivo está activo el Led verde está activado
        NETSTACK_MAC.on();
      }

      /*
       * Next, switch between normal and very sleepy mode depending on config,
       * send notifications to observers as required.
       */
      if(state == STATE_NOTIFY_OBSERVERS) {							    //Si el estado es notificar observadores
        REST.notify_subscribers(&readings_resource);					//Envia a traves de REST un recurso "readings"
        state = STATE_NORMAL;										    //Cambia a estado normal
        printf("Paso a estado normal\n");
      }

      if(state == STATE_NORMAL) {									          //Si está en estado normal
        if(stimer_expired(&st_duration)) {							    	  //Y el timer de nueva configuracion ha expirado
          stimer_set(&st_duration, config.duration);						  //Se fija el timer de nuevo
          if(config.mode == VERY_SLEEPY_MODE_ON) {							  //Y si la configuracion nueva muestra modo Sleepy
          	printf("Paso a modo de sueño profundo\n");
            switch_to_very_sleepy();								          //Pasamos a Modo Sleepy
          }
        }
      } else if(state == STATE_VERY_SLEEPY) {						    //Si estamos en modo Sleepy
        if(stimer_expired(&st_interval)) {							    //Y el timer de intervalo ha expirado
        	printf("Paso a modo normal, expirado tiempo de intervalo\n");
          switch_to_normal();										            //Cambia a normal
        }
      }

      if(mac_keep_on == MAC_CAN_BE_TURNED_OFF && state == STATE_VERY_SLEEPY) {	//Si la MAC está desactivada y estamos en Sleepy
        leds_off(LEDS_GREEN);
        NETSTACK_MAC.off(0);
      } else {
        leds_on(LEDS_GREEN);
        NETSTACK_MAC.on();
      }

      /* Schedule next pass */
      etimer_set(&et_periodic, PERIODIC_INTERVAL);					//Fija el próximo intervalo de un segundo
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

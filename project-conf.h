/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

#include "net/ipv6/multicast/uip-mcast6-engines.h"

/* Change this to switch engines. Engine codes in uip-mcast6-engines.h */
#define UIP_MCAST6_CONF_ENGINE UIP_MCAST6_ENGINE_ROLL_TM
/* For Imin: Use 16 over NullRDC, 64 over Contiki MAC */
#define ROLL_TM_CONF_IMIN_1         64

#undef UIP_CONF_IPV6_RPL
#undef UIP_CONF_ND6_SEND_RA
#undef UIP_CONF_ROUTER
#undef NETSTACK_CONF_WITH_IPV6	
#define UIP_CONF_ND6_SEND_RA         0
#define UIP_CONF_ROUTER              1			//El nodo actúa como Router
#define UIP_CONF_IPV6_RPL			 1			//RPL es usado para rutar Ipv6
#define UIP_MCAST6_ROUTE_CONF_ROUTES 1			//
#define NETSTACK_CONF_WITH_IPV6      1			//Especifica que ipv6 debe ser usado

#undef UIP_CONF_TCP
#define UIP_CONF_TCP 0

/* Code/RAM footprint savings so that things will fit on our device */
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES
#define NBR_TABLE_CONF_MAX_NEIGHBORS  10
#define UIP_CONF_MAX_ROUTES           10

/*---------------------------------------------------------------------------*/
/* Change to match your configuration */
#define IEEE802154_CONF_PANID            0xABCD
#define RF_CORE_CONF_CHANNEL                 25
/*---------------------------------------------------------------------------*/
/* Enable the ROM bootloader */
#define ROM_BOOTLOADER_ENABLE                 1
#define BOARD_CONF_DEBUGGER_DEVPACK			  1
/* Disable button shutdown functionality */
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN    1
/*---------------------------------------------------------------------------*/
/* For very sleepy operation */
#define RF_BLE_CONF_ENABLED                   0
#define UIP_DS6_CONF_PERIOD        CLOCK_SECOND
#define RPL_CONF_LEAF_ONLY                    1

#define REST_MAX_CHUNK_SIZE     128
/*
 * We'll fail without RPL probing, so turn it on explicitly even though it's
 * on by default
 */
#define RPL_CONF_WITH_PROBING                 1
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/

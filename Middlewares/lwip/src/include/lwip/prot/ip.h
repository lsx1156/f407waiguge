/**
 * @file
 * Common IP protocol definitions
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#ifndef LWIP_HDR_PROT_IP_H
#define LWIP_HDR_PROT_IP_H

#include "lwip/opt.h"
#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* This is passed as the destination address to ip_output_if (not
   to ip_output), meaning that an IP header already is constructed
   in the pbuf. This is used when TCP retransmits. */
#define LWIP_IP_HDRINCL  NULL

#if LWIP_NETIF_USE_HINTS
#define IP_PCB_NETIFHINT ;struct netif_hint netif_hints
#else /* LWIP_NETIF_USE_HINTS */
#define IP_PCB_NETIFHINT
#endif /* LWIP_NETIF_USE_HINTS */

/** This is the common part of all PCB types. It needs to be at the
   beginning of a PCB type definition. It is located here so that
   changes to this common part are made in one location instead of
   having to change all PCB structs. */
#define IP_PCB                             \
  /* ip addresses in network byte order */ \
  ip_addr_t local_ip;                      \
  ip_addr_t remote_ip;                     \
  /* Bound netif index */                  \
  u8_t netif_idx;                          \
  /* Socket options */                     \
  u8_t so_options;                         \
  /* Type Of Service */                    \
  u8_t tos;                                \
  /* Time To Live */                       \
  u8_t ttl                                 \
  /* link layer address resolution hint */ \
  IP_PCB_NETIFHINT

struct ip_pcb {
/* Common members of all PCB types */
  IP_PCB;
};

/*
 * Option flags per-socket. These are the same like SO_XXX in sockets.h
 */
#define SOF_REUSEADDR     0x04U  /* allow local address reuse */
#define SOF_KEEPALIVE     0x08U  /* keep connections alive */
#define SOF_BROADCAST     0x20U  /* permit to send and to receive broadcast messages (see IP_SOF_BROADCAST option) */

/* These flags are inherited (e.g. from a listen-pcb to a connection-pcb): */
#define SOF_INHERITED   (SOF_REUSEADDR|SOF_KEEPALIVE)

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_IP_H */

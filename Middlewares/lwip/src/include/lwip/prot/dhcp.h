/**
 * @file
 * DHCP protocol definitions
 */

/*
 * Copyright (c) 2001-2004 Leon Woestenberg <leon.woestenberg@axon.tv>
 * Copyright (c) 2001-2004 Axon Digital Design B.V., The Netherlands.
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
 * Author: Leon Woestenberg <leon.woestenberg@axon.tv>
 *
 */

#ifndef LWIP_HDR_PROT_DHCP_H
#define LWIP_HDR_PROT_DHCP_H

#include "lwip/opt.h"
#include "lwip/prot/ip4.h"
#include "lwip/prot/iana.h"

#ifdef __cplusplus
extern "C" {
#endif

/** DHCP option lengths */
#define DHCP_OPTIONS_LEN 312U

/** minimum length of DHCP request, options included */
#define DHCP_MIN_REQUEST_LEN 44U

#define DHCP_CLIENT_PORT  LWIP_IANA_PORT_DHCP_CLIENT
#define DHCP_SERVER_PORT  LWIP_IANA_PORT_DHCP_SERVER

/** DHCP message op codes */
#define DHCP_BOOTREQUEST  1
#define DHCP_BOOTREPLY    2

/** DHCP message types */
#define DHCP_DISCOVER     1
#define DHCP_OFFER        2
#define DHCP_REQUEST      3
#define DHCP_DECLINE      4
#define DHCP_ACK          5
#define DHCP_NAK          6
#define DHCP_RELEASE      7
#define DHCP_INFORM       8

/** DHCP magic cookie */
#define DHCP_MAGIC_COOKIE 0x63825363UL

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
/** DHCP message */
struct dhcp_msg {
  PACK_STRUCT_FLD_8(u8_t op);
  PACK_STRUCT_FLD_8(u8_t htype);
  PACK_STRUCT_FLD_8(u8_t hlen);
  PACK_STRUCT_FLD_8(u8_t hops);
  PACK_STRUCT_FIELD(u32_t xid);
  PACK_STRUCT_FIELD(u16_t secs);
  PACK_STRUCT_FIELD(u16_t flags);
  PACK_STRUCT_FLD_S(ip4_addr_p_t ciaddr);
  PACK_STRUCT_FLD_S(ip4_addr_p_t yiaddr);
  PACK_STRUCT_FLD_S(ip4_addr_p_t siaddr);
  PACK_STRUCT_FLD_S(ip4_addr_p_t giaddr);
  PACK_STRUCT_FLD_8(u8_t chaddr[16]);
  PACK_STRUCT_FLD_8(u8_t sname[64]);
  PACK_STRUCT_FLD_8(u8_t file[128]);
  PACK_STRUCT_FLD_8(u8_t options[DHCP_OPTIONS_LEN]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_DHCP_H */

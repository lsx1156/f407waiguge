/**
 * @file
 * Ethernet protocol definitions
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

#ifndef LWIP_HDR_PROT_ETHERNET_H
#define LWIP_HDR_PROT_ETHERNET_H

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Ethernet frame types */
#define ETHTYPE_IP        0x0800U
#define ETHTYPE_ARP       0x0806U
#define ETHTYPE_VLAN      0x8100U
#define ETHTYPE_IPV6      0x86DDU
#define ETHTYPE_PPPOEDISC 0x8863U
#define ETHTYPE_PPPOE     0x8864U
#define ETHTYPE_WAKE_ON_LAN 0x0842U
#define ETHTYPE_MPLS      0x8847U
#define ETHTYPE_MPLS_MCAST 0x8848U
#define ETHTYPE_ETHERCAT  0x88A4U
#define ETHTYPE_QINQ      0x88A8U
#define ETHTYPE_PROFINET  0x8892U
#define ETHTYPE_ETHERLINK 0x8893U
#define ETHTYPE_ETHERLINK_OAM 0x8906U
#define ETHTYPE_JUMBO     0x8870U

/** The size of an Ethernet MAC address */
#ifndef ETH_HWADDR_LEN
#define ETH_HWADDR_LEN    6U
#endif
/** The size of an Ethernet MAC address (alias) */
#define ETH_ADDRLEN       ETH_HWADDR_LEN

/** Maximum size of an Ethernet frame payload */
#define ETH_MAX_PAYLOAD_LEN 1500U

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct eth_addr {
  PACK_STRUCT_FLD_8(u8_t addr[ETH_HWADDR_LEN]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct eth_hdr {
  PACK_STRUCT_FLD_S(struct eth_addr dest);
  PACK_STRUCT_FLD_S(struct eth_addr src);
  PACK_STRUCT_FIELD(u16_t type);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct eth_vlan_hdr {
  PACK_STRUCT_FLD_S(struct eth_addr dest);
  PACK_STRUCT_FLD_S(struct eth_addr src);
  PACK_STRUCT_FIELD(u16_t type);
  PACK_STRUCT_FIELD(u16_t prio_vid);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#define ETH_VLAN_PCP(vlan_hdr) (lwip_ntohs((vlan_hdr)->prio_vid) >> 13)
#define ETH_VLAN_VID(vlan_hdr) (lwip_ntohs((vlan_hdr)->prio_vid) & 0xFFF)
#define VLAN_TCI_ID(tci)       ((tci) & 0xFFFU)
#define VLAN_TCI_PRI(tci)      (((tci) >> 13) & 0x7U)
#define VLAN_TCI_DEI(tci)      (((tci) >> 12) & 0x1U)
#define VLAN_TCI_CFI(tci)      VLAN_TCI_DEI(tci)
#define VLAN_TCI_PID(tci)      (((tci) >> 12) & 0x1U)

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_ETHERNET_H */

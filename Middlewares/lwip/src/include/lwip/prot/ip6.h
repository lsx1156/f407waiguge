/**
 * @file
 * IPv6 protocol definitions
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

#ifndef LWIP_HDR_PROT_IP6_H
#define LWIP_HDR_PROT_IP6_H

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* This is the packed version of ip6_addr_t,
   used in network headers that are itself packed */
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct ip6_addr_packed {
  PACK_STRUCT_FIELD(u32_t addr[4]);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif
typedef struct ip6_addr_packed ip6_addr_p_t;

/** IPv6 header */
#define IP6_HLEN 40

/* Next header numbers */
#define IP6_NEXTH_HOPBYHOP      0U
#define IP6_NEXTH_TCP           6U
#define IP6_NEXTH_UDP           17U
#define IP6_NEXTH_ENCAP         41U
#define IP6_NEXTH_ROUTING       43U
#define IP6_NEXTH_FRAGMENT      44U
#define IP6_NEXTH_ICMP6         58U
#define IP6_NEXTH_NONE          59U
#define IP6_NEXTH_DESTOPT       60U
#define IP6_NEXTH_UDPLITE       136U

/* The IPv6 header. */
#define IP6_VTC_FL_ECN_MASK     0x30000000U
#define IP6_VTC_FL_ECN_SHIFT    29
#define IP6_VTC_FL_DSCP_MASK    0x0FC00000U
#define IP6_VTC_FL_DSCP_SHIFT   22

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
struct ip6_hdr {
  /** version / traffic class / flow label */
  PACK_STRUCT_FIELD(u32_t _v_tc_fl);
  /** payload length */
  PACK_STRUCT_FIELD(u16_t _plen);
  /** next header */
  PACK_STRUCT_FLD_8(u8_t _nexth);
  /** hop limit */
  PACK_STRUCT_FLD_8(u8_t _hoplim);
  /** source and destination IP addresses */
  PACK_STRUCT_FLD_S(ip6_addr_p_t src);
  PACK_STRUCT_FLD_S(ip6_addr_p_t dest);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/* Macros for getting / setting IPv6 header fields. */
#define IP6H_V(hdr)               ((lwip_ntohl((hdr)->_v_tc_fl) >> 28) & 0x0f)
#define IP6H_TC(hdr)              ((lwip_ntohl((hdr)->_v_tc_fl) >> 20) & 0xff)
#define IP6H_FL(hdr)              (lwip_ntohl((hdr)->_v_tc_fl) & 0xfffff)
#define IP6H_PLEN(hdr)            (lwip_ntohs((hdr)->_plen))
#define IP6H_NEXTH(hdr)           ((hdr)->_nexth)
#define IP6H_NEXTH_SET(hdr, val)  (hdr)->_nexth = (val)
#define IP6H_HOPLIM(hdr)          ((hdr)->_hoplim)
#define IP6H_HOPLIM_SET(hdr, val) (hdr)->_hoplim = (u8_t)(val)
#define IP6H_VTCFL_SET(hdr, v, tc, fl) (hdr)->_v_tc_fl = (lwip_htonl((((u32_t)(v) << 28) | ((u32_t)(tc) << 20) | (u32_t)(fl))))
#define IP6H_PLEN_SET(hdr, plen)  (hdr)->_plen = lwip_htons(plen)

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_IP6_H */

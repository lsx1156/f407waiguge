/**
 * @file
 * ICMPv6 protocol definitions
 */

/*
 * Copyright (c) 2010 Inico Technologies Ltd.
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
 * Author: Ivan Delamer <delamer@inicotech.com>
 *
 */

#ifndef LWIP_HDR_PROT_ICMP6_H
#define LWIP_HDR_PROT_ICMP6_H

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/** ICMP type codes */
#define ICMP6_TYPE_DEST_UNREACHABLE       1U  /* Destination unreachable */
#define ICMP6_TYPE_PACKET_TOO_BIG         2U  /* Packet too big */
#define ICMP6_TYPE_TIME_EXCEEDED          3U  /* Time exceeded */
#define ICMP6_TYPE_PARAMETER_PROBLEM      4U  /* Parameter problem */
#define ICMP6_TYPE_PRIVATE_EXPERIMENTATION 100U
#define ICMP6_TYPE_PRIVATE_EXPERIMENTATION2 200U
#define ICMP6_TYPE_ECHO_REQUEST         128U  /* Echo request */
#define ICMP6_TYPE_ECHO_REPLY           129U  /* Echo reply */
#define ICMP6_TYPE_ROUTER_SOLICITATION  133U  /* Router solicitation */
#define ICMP6_TYPE_ROUTER_ADVERTISEMENT 134U  /* Router advertisement */
#define ICMP6_TYPE_NEIGHBOR_SOLICITATION 135U /* Neighbor solicitation */
#define ICMP6_TYPE_NEIGHBOR_ADVERTISEMENT 136U /* Neighbor advertisement */
#define ICMP6_TYPE_REDIRECT             137U  /* Redirect */
#define ICMP6_TYPE_MLD_QUERY            130U  /* Multicast listener query */
#define ICMP6_TYPE_MLD_REPORT           131U  /* Multicast listener report */
#define ICMP6_TYPE_MLD_DONE             132U  /* Multicast listener done */
#define ICMP6_TYPE_MLD2_REPORT          143U  /* Multicast listener report v2 */
#define ICMP6_TYPE_RDNSS                134U  /* RDNSS option (RFC 8106) */

/** ICMP6 destination unreachable codes (enum as used by lwIP API) */
enum icmp6_dur_code {
  ICMP6_DUR_NOROUTE         = 0,
  ICMP6_DUR_PROHIBITED      = 1,
  ICMP6_DUR_SCOPE           = 2,
  ICMP6_DUR_ADDRESS         = 3,
  ICMP6_DUR_PORT            = 4,
  ICMP6_DUR_POLICY          = 5,
  ICMP6_DUR_REJECTROUTE     = 6
};

/** ICMP6 time exceeded codes (enum as used by lwIP API) */
enum icmp6_te_code {
  ICMP6_TE_HOPCOUNT         = 0,
  ICMP6_TE_FRAGTIME         = 1
};

/** ICMP6 parameter problem codes (enum as used by lwIP API) */
enum icmp6_pp_code {
  ICMP6_PP_FIELD             = 0,
  ICMP6_PP_HEADER            = 1,
  ICMP6_PP_OPTION            = 2
};

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
/** basic ICMPv6 header */
struct icmp6_hdr {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FIELD(u32_t data);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
/** ICMPv6 echo header */
struct icmp6_echo_hdr {
  PACK_STRUCT_FLD_8(u8_t type);
  PACK_STRUCT_FLD_8(u8_t code);
  PACK_STRUCT_FIELD(u16_t chksum);
  PACK_STRUCT_FIELD(u16_t id);
  PACK_STRUCT_FIELD(u16_t seqno);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#define ICMP6H_TYPE(hdr) ((hdr)->type)
#define ICMP6H_CODE(hdr) ((hdr)->code)
#define ICMP6H_TYPE_SET(hdr, t) ((hdr)->type = (t))
#define ICMP6H_CODE_SET(hdr, c) ((hdr)->code = (c))

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_ICMP6_H */

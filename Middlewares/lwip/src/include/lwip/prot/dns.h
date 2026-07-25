/**
 * @file
 * DNS protocol definitions
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

#ifndef LWIP_HDR_PROT_DNS_H
#define LWIP_HDR_PROT_DNS_H

#include "lwip/opt.h"
#include "lwip/prot/iana.h"

#ifdef __cplusplus
extern "C" {
#endif

/** DNS server port number */
#define DNS_SERVER_PORT       LWIP_IANA_PORT_DNS
/** DNS header length */
#define DNS_HLEN              12

/** DNS flags (first byte) */
#define DNS_FLAG1_RESPONSE        0x80
#define DNS_FLAG1_OPCODE_STATUS   0x10
#define DNS_FLAG1_OPCODE_IQUERY   0x08
#define DNS_FLAG1_OPCODE_MASK     0x78
#define DNS_FLAG1_AUTHORITATIVE   0x04
#define DNS_FLAG1_TRUNC           0x02
#define DNS_FLAG1_RECURSE         0x01
/** DNS flags (second byte) */
#define DNS_FLAG2_RECURSE_AVAIL   0x80
#define DNS_FLAG2_ERR_MASK        0x0f

/* DNS OPCODE values (in DNS_FLAG1_OPCODE_*) */
#define DNS_OPCODE_QUERY          0x00
#define DNS_OPCODE_IQUERY         0x01
#define DNS_OPCODE_STATUS         0x02

/* DNS reply codes */
#define DNS_REPLY_CODE_NO_ERR     0
#define DNS_REPLY_CODE_FORMAT_ERR 1
#define DNS_REPLY_CODE_SERVER_ERR 2
#define DNS_REPLY_CODE_NAME_ERR   3
#define DNS_REPLY_CODE_NOT_IMPL   4
#define DNS_REPLY_CODE_REFUSED    5

/* DNS resource record types */
#define DNS_RRTYPE_A              1
#define DNS_RRTYPE_NS             2
#define DNS_RRTYPE_CNAME          5
#define DNS_RRTYPE_SOA            6
#define DNS_RRTYPE_PTR            12
#define DNS_RRTYPE_MX             15
#define DNS_RRTYPE_TXT            16
#define DNS_RRTYPE_AAAA           28
#define DNS_RRTYPE_SRV            33
#define DNS_RRTYPE_ANY            255

/* DNS resource record classes */
#define DNS_RRCLASS_IN            1
#define DNS_RRCLASS_ANY           255

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
/** DNS message header */
struct dns_hdr {
  PACK_STRUCT_FIELD(u16_t id);
  PACK_STRUCT_FLD_8(u8_t flags1);
  PACK_STRUCT_FLD_8(u8_t flags2);
  PACK_STRUCT_FIELD(u16_t numquestions);
  PACK_STRUCT_FIELD(u16_t numanswers);
  PACK_STRUCT_FIELD(u16_t numauthrr);
  PACK_STRUCT_FIELD(u16_t numextrarr);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

/** Convenience flag aliases (used by lwIP) */
#define DNS_FLAG1_RD              DNS_FLAG1_RECURSE

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_DNS_H */

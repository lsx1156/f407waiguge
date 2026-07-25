/**
 * @file
 * IGMP protocol definitions
 */

/*
 * Copyright (c) 2002 CITEL Technologies Ltd.
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
 * Author: Steve Reynolds
 *
 */

#ifndef LWIP_HDR_PROT_IGMP_H
#define LWIP_HDR_PROT_IGMP_H

#include "lwip/opt.h"
#include "lwip/prot/ip4.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IGMP_MEMB_QUERY       0x11U  /* Membership query    */
#define IGMP_V1_MEMB_REPORT   0x12U  /* Ver. 1 membership report */
#define IGMP_V2_MEMB_REPORT   0x16U  /* Ver. 2 membership report */
#define IGMP_LEAVE_GROUP      0x17U  /* Leave group message */
#define IGMP_V3_MEMB_REPORT   0x22U  /* Ver. 3 membership report */

#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/bpstruct.h"
#endif
PACK_STRUCT_BEGIN
/** IGMP message */
struct igmp_msg {
  PACK_STRUCT_FLD_8(u8_t igmp_msgtype);
  PACK_STRUCT_FLD_8(u8_t igmp_maxresp);
  PACK_STRUCT_FIELD(u16_t igmp_checksum);
  PACK_STRUCT_FLD_S(ip4_addr_p_t igmp_group_address);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END
#ifdef PACK_STRUCT_USE_INCLUDES
#  include "arch/epstruct.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_IGMP_H */

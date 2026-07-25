/**
 * @file
 * AutoIP protocol definitions
 *
 * The AutoIP state machine constants and the link-local address range
 * (RFC 3927). The per-netif AutoIP state is kept in "struct autoip"
 * declared in "lwip/autoip.h".
 */

/*
 * Copyright (c) 2007 Dominik Spies <kontakt@dspies.de>
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
 * Author: Dominik Spies <kontakt@dspies.de>
 *
 */

#ifndef LWIP_HDR_PROT_AUTOIP_H
#define LWIP_HDR_PROT_AUTOIP_H

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* AutoIP states (RFC 3927) */
#define AUTOIP_STATE_OFF         0
#define AUTOIP_STATE_PROBING     1
#define AUTOIP_STATE_ANNOUNCING  2
#define AUTOIP_STATE_BOUND       3

/* RFC 3927 link-local address range 169.254.0.0 - 169.254.255.255.
 * The values are in host byte order; lwip_htonl() converts them when needed. */
/* 169.254.0.0 */
#define AUTOIP_RANGE_START (0xA9FE0000UL)
/* 169.254.255.255 */
#define AUTOIP_RANGE_END   (0xA9FEFFFFUL)

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_AUTOIP_H */

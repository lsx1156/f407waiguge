/**
 * @file
 * IANA assigned numbers (RFC 1700 and successors)
 *
 * These are the IANA assigned numbers that are used in lwIP:
 * - protocol numbers for IP
 * - port numbers for TCP/UDP
 * - hardware type numbers for ARP
 */

/*
 * Copyright (c) 2017 Dirk Ziegelmeier
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
 * Author: Dirk Ziegelmeier <dziegelmeier@de.pepperl-fuchs.com>
 *
 */

#ifndef LWIP_HDR_PROT_IANA_H
#define LWIP_HDR_PROT_IANA_H

#include "lwip/opt.h"

#ifdef __cplusplus
extern "C" {
#endif

/* port numbers for TCP/UDP (RFC 6335) */
#define LWIP_IANA_PORT_TCPMUX      1U
#define LWIP_IANA_PORT_ECHO        7U
#define LWIP_IANA_PORT_DISCARD     9U
#define LWIP_IANA_PORT_SYSTAT      11U
#define LWIP_IANA_PORT_DAYTIME     13U
#define LWIP_IANA_PORT_QOTD        17U
#define LWIP_IANA_PORT_CHARGEN     19U
#define LWIP_IANA_PORT_FTP_DATA    20U
#define LWIP_IANA_PORT_FTP         21U
#define LWIP_IANA_PORT_TELNET      23U
#define LWIP_IANA_PORT_SMTP        25U
#define LWIP_IANA_PORT_TIME        37U
#define LWIP_IANA_PORT_NAME        42U
#define LWIP_IANA_PORT_WHOIS       43U
#define LWIP_IANA_PORT_DOMAIN      53U
#define LWIP_IANA_PORT_DNS         53U /* alias */
#define LWIP_IANA_PORT_TFTP        69U
#define LWIP_IANA_PORT_HTTP        80U
#define LWIP_IANA_PORT_KERBEROS    88U
#define LWIP_IANA_PORT_RTELNET     107U
#define LWIP_IANA_PORT_POP2        109U
#define LWIP_IANA_PORT_POP3        110U
#define LWIP_IANA_PORT_SUNRPC      111U
#define LWIP_IANA_PORT_AUTH        113U
#define LWIP_IANA_PORT_SFTP        115U
#define LWIP_IANA_PORT_NNTP        119U
#define LWIP_IANA_PORT_NTP         123U
#define LWIP_IANA_PORT_NETBIOS     137U
#define LWIP_IANA_PORT_NETBIOS_DGM 138U
#define LWIP_IANA_PORT_NETBIOS_SSN 139U
#define LWIP_IANA_PORT_IMAP        143U
#define LWIP_IANA_PORT_SNMP        161U
#define LWIP_IANA_PORT_SNMP_TRAP   162U
#define LWIP_IANA_PORT_BGP         179U
#define LWIP_IANA_PORT_IRC         194U
#define LWIP_IANA_PORT_IMAP3       220U
#define LWIP_IANA_PORT_LDAP        389U
#define LWIP_IANA_PORT_HTTPS       443U
#define LWIP_IANA_PORT_SMTPS       465U
#define LWIP_IANA_PORT_DHCP_CLIENT 68U
#define LWIP_IANA_PORT_DHCP_SERVER 67U

/* protocol numbers (RFC 790) */
#define LWIP_IANA_PROTO_ICMP       1U
#define LWIP_IANA_PROTO_IGMP       2U
#define LWIP_IANA_PROTO_TCP        6U
#define LWIP_IANA_PROTO_UDP        17U
#define LWIP_IANA_PROTO_ICMP6      58U
#define LWIP_IANA_PROTO_UDPLITE    136U

/* hardware type numbers for ARP (RFC 826) */
#define LWIP_IANA_HWTYPE_ETHERNET  1U

#ifdef __cplusplus
}
#endif

#endif /* LWIP_HDR_PROT_IANA_H */

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define SYS_LIGHTWEIGHT_PROT        1

#define NO_SYS                      1

#define LWIP_DISABLE_TCP_SANITY_CHECKS 1

#define MEM_ALIGNMENT               4

#define MEM_SIZE                    (64*1024)

#define MEM_LIBC_MALLOC             1

#define LWIP_ALLOW_MEM_FREE         1

#define MEMP_NUM_PBUF               32
#define MEMP_NUM_UDP_PCB            4
#define MEMP_NUM_TCP_PCB            0
#define MEMP_NUM_TCP_PCB_LISTEN     0
#define MEMP_NUM_TCP_SEG            0
#define MEMP_NUM_SYS_TIMEOUT        8

#define PBUF_POOL_SIZE              32
#define PBUF_POOL_BUFSIZE           LWIP_MEM_ALIGN_SIZE(TCP_MSS+40+PBUF_LINK_ENCAPSULATION_HLEN+PBUF_LINK_HLEN)

#define LWIP_TCP                    0
#define TCP_TTL                     255

#define TCP_QUEUE_OOSEQ             0

#define TCP_MSS                     (1500 - 40)

#define TCP_SND_BUF                 (2*TCP_MSS)

#define TCP_SNDQUEUELOWAT           4

#define TCP_WND                     (2*TCP_MSS)

#define LWIP_ICMP                   1

#define LWIP_DHCP                   0

#define LWIP_UDP                    1
#define UDP_TTL                     255

#define LWIP_STATS                  0
#define LWIP_PROVIDE_ERRNO          1

#define LWIP_NETIF_LINK_CALLBACK    1

#define CHECKSUM_BY_HARDWARE 

#ifdef CHECKSUM_BY_HARDWARE
  #define CHECKSUM_GEN_IP           0
  #define CHECKSUM_GEN_UDP          0
  #define CHECKSUM_GEN_TCP          0 
  #define CHECKSUM_CHECK_IP         0
  #define CHECKSUM_CHECK_UDP        0
  #define CHECKSUM_CHECK_TCP        0
  #define CHECKSUM_GEN_ICMP         0
#else
  #define CHECKSUM_GEN_IP           1
  #define CHECKSUM_GEN_UDP          1
  #define CHECKSUM_GEN_TCP          1
  #define CHECKSUM_CHECK_IP         1
  #define CHECKSUM_CHECK_UDP        1
  #define CHECKSUM_CHECK_TCP        1
  #define CHECKSUM_GEN_ICMP         1
#endif

#define LWIP_NETCONN                0

#define LWIP_SOCKET                 0

#define PPP_SUPPORT                 0

#define LWIP_ARP                    1

#define LWIP_ETHERNET               1

#define LWIP_IPV4                   1

#define LWIP_IPV6                   0

#define ETHARP_SUPPORT_VLAN         0

#define IP_REASSEMBLY               0
#define IP_FRAG                     0

#define LWIP_ALTCP                  0

#define ARP_QUEUEING                0

#define LWIP_RAW                    0

#define LWIP_SNMP                   0

#define LWIP_IGMP                   0

#define LWIP_TIMEVAL_PRIVATE        0

#define TCPH_SET_FLAG(phdr, flags)    TCPH_SETFLAG(phdr, flags)

#define IPADDR_WORDALIGNED_COPY_TO_IP4_ADDR_T(dest, src)  SMEMCPY((dest), (src), sizeof(ip4_addr_t))
#define IPADDR_WORDALIGNED_COPY_FROM_IP4_ADDR_T(dest, src) SMEMCPY((dest), (src), sizeof(ip4_addr_t))

#define eth_type_vlan(type) ((type) == PP_HTONS(ETHTYPE_VLAN))

#ifdef __cplusplus
extern "C" {
#endif
int printf(const char *format, ...);
#ifdef __cplusplus
}
#endif
#define LWIP_PLATFORM_DIAG(x)   do { printf x; } while(0)

#endif

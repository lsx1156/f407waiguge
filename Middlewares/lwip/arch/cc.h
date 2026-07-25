#ifndef __CC_H__
#define __CC_H__

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

typedef int sys_prot_t;

#define BYTE_ORDER LITTLE_ENDIAN

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif



#if defined (__GNUC__) & !defined (__CC_ARM)

#define LWIP_TIMEVAL_PRIVATE 0
/* STM32无POSIX库，注释掉sys/time.h避免编译错误 */
/* #include <sys/time.h> */

#endif

#if defined (__ICCARM__)

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT 
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_USE_INCLUDES

#elif (__ARMCC_VERSION >= 6010050)

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__((packed))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#elif defined (__GNUC__)

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#elif defined (__CC_ARM)

#define PACK_STRUCT_BEGIN __packed
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#elif defined (__TASKING__)

#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

#endif

#define LWIP_PLATFORM_ASSERT(x) do {printf("Assertion \"%s\" failed at line %d in %s\n", \
                                     x, __LINE__, __FILE__); } while(0)

#define LWIP_RAND() ((u32_t)rand())

#endif
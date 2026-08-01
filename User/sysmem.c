/**
 ******************************************************************************
 * @file      sysmem.c
 * @brief     System Memory calls file (Keil/GCC compatible)
 ******************************************************************************
 */

/* Includes */
#include <errno.h>
#include <stdint.h>

#if defined(__GNUC__) && !defined(__CC_ARM) && !defined(__ARMCC_VERSION)

/**
 * Pointer to the current high watermark of the heap usage
 */
static uint8_t *__sbrk_heap_end = NULL;

void *_sbrk(ptrdiff_t incr)
{
  extern uint8_t _end;
  extern uint8_t _estack;
  extern uint32_t _Min_Stack_Size;
  const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint8_t *max_heap = (uint8_t *)stack_limit;
  uint8_t *prev_heap_end;

  if (NULL == __sbrk_heap_end)
  {
    __sbrk_heap_end = &_end;
  }

  if (__sbrk_heap_end + incr > max_heap)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev_heap_end = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}

#endif /* __GNUC__ */

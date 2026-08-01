#include "lwip/err.h"
#include "lwip/sys.h"
#include <string.h>
#include "stm32f4xx_hal.h"

/* ── SYS_LIGHTWEIGHT_PROT=1 时，无论 NO_SYS 为何值都需实现 protect/unprotect ── */
#if SYS_LIGHTWEIGHT_PROT

sys_prot_t sys_arch_protect(void)
{
    sys_prot_t pri = __get_PRIMASK();
    __disable_irq();
    return pri;
}

void sys_arch_unprotect(sys_prot_t pval)
{
    if (!(pval & 1)) {
        __enable_irq();
    }
}

#endif /* SYS_LIGHTWEIGHT_PROT */

#if !NO_SYS

err_t sys_mutex_new(sys_mutex_t *mutex)
{
    *mutex = NULL;
    return ERR_OK;
}

void sys_mutex_free(sys_mutex_t mutex)
{
    UNUSED(mutex);
}

void sys_mutex_lock(sys_mutex_t mutex)
{
    UNUSED(mutex);
}

void sys_mutex_unlock(sys_mutex_t mutex)
{
    UNUSED(mutex);
}

err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
    UNUSED(count);
    *sem = NULL;
    return ERR_OK;
}

void sys_sem_free(sys_sem_t sem)
{
    UNUSED(sem);
}

u32_t sys_arch_sem_wait(sys_sem_t sem, u32_t timeout)
{
    UNUSED(sem);
    UNUSED(timeout);
    return 0;
}

void sys_sem_signal(sys_sem_t sem)
{
    UNUSED(sem);
}

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    UNUSED(size);
    *mbox = NULL;
    return ERR_OK;
}

void sys_mbox_free(sys_mbox_t mbox)
{
    UNUSED(mbox);
}

void sys_mbox_post(sys_mbox_t mbox, void *msg)
{
    UNUSED(mbox);
    UNUSED(msg);
}

u32_t sys_arch_mbox_fetch(sys_mbox_t mbox, void **msg, u32_t timeout)
{
    UNUSED(mbox);
    UNUSED(msg);
    UNUSED(timeout);
    return 0;
}

u32_t sys_arch_mbox_tryfetch(sys_mbox_t mbox, void **msg)
{
    return sys_arch_mbox_fetch(mbox, msg, 0);
}

sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
    UNUSED(name);
    UNUSED(thread);
    UNUSED(arg);
    UNUSED(stacksize);
    UNUSED(prio);
    return NULL;
}

void sys_msleep(u32_t ms)
{
    UNUSED(ms);
}

#endif /* !NO_SYS */

/* ── sys_now() 在 NO_SYS 和 !NO_SYS 模式下都需要 ── */
u32_t sys_now(void)
{
    return HAL_GetTick();
}
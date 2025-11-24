/*
 * Generic RPMsg-Lite platform hooks for RT-Thread targets where no SoC
 * implementation is provided. The functions implement minimal bookkeeping so
 * the stack can compile and basic transport hooks can be wired later.
 */

#include <stdint.h>

#include "rpmsg_platform.h"
#include "rpmsg_env.h"

static int32_t isr_counter;
static int32_t disable_counter;
static void *platform_lock;

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    env_register_isr(vector_id, isr_data);

    env_lock_mutex(platform_lock);
    isr_counter++;
    env_unlock_mutex(platform_lock);

    return 0;
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
    env_lock_mutex(platform_lock);

    if (isr_counter > 0)
    {
        isr_counter--;
    }

    env_unregister_isr(vector_id);

    env_unlock_mutex(platform_lock);

    return 0;
}

int32_t platform_interrupt_enable(uint32_t vector_id)
{
    (void)vector_id;

    env_lock_mutex(platform_lock);
    if (disable_counter > 0)
    {
        disable_counter--;
    }
    env_unlock_mutex(platform_lock);

    return (int32_t)vector_id;
}

int32_t platform_interrupt_disable(uint32_t vector_id)
{
    (void)vector_id;

    env_lock_mutex(platform_lock);
    disable_counter++;
    env_unlock_mutex(platform_lock);

    return (int32_t)vector_id;
}

int32_t platform_in_isr(void)
{
    return 0;
}

void platform_notify(uint32_t vector_id)
{
    /*
     * Generic RT-Thread builds typically deliver notifications through
     * transport-specific hooks that are not available here. Provide a safe
     * placeholder to keep compilation working.
     */
    (void)vector_id;
}

void platform_time_delay(uint32_t num_msec)
{
    env_sleep_msec(num_msec);
}

void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags)
{
    (void)vrt_addr;
    (void)phy_addr;
    (void)size;
    (void)flags;
}

void platform_cache_all_flush_invalidate(void)
{
}

void platform_cache_disable(void)
{
}

uint32_t platform_vatopa(void *addr)
{
    return (uint32_t)(uintptr_t)addr;
}

void *platform_patova(uint32_t addr)

{
    return (void *)(uintptr_t)addr;
}

int32_t platform_init(void)
{
    isr_counter     = 0;
    disable_counter = 0;

    env_create_mutex(&platform_lock, 1);

    return 0;
}

int32_t platform_deinit(void)
{
    env_delete_mutex(platform_lock);
    platform_lock = 0;

    return 0;
}

/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-12-08     Bernard      fix the issue of _timevalue.tv_usec initialization,
 *                             which found by Rob <rdent@iinet.net.au>
 */

#include <rtdevice.h>
#include <time.h>
#include "clock_time.h"

static struct timeval _timevalue;
int clock_time_system_init()
{
    time_t time;
    rt_tick_t tick;
    rt_device_t device;

    time = 0;
    device = rt_device_find("rtc");
    if (device != RT_NULL)
    {
        /* get realtime seconds */
        rt_device_control(device, RT_DEVICE_CTRL_RTC_GET_TIME, &time);
    }

    /* get tick */
    tick = rt_tick_get();

    _timevalue.tv_usec = (tick%RT_TICK_PER_SECOND) * MICROSECOND_PER_TICK;
    _timevalue.tv_sec = time - tick/RT_TICK_PER_SECOND - 1;

    return 0;
}
INIT_COMPONENT_EXPORT(clock_time_system_init);

int clock_time_to_tick(const struct timespec *time)
{
    int tick;
    long nsecond, second;
    struct timespec tp;

    RT_ASSERT(time != RT_NULL);

    tick = RT_WAITING_FOREVER;
    if (time != RT_NULL)
    {
        /* get current tp */
        clock_gettime(CLOCK_REALTIME, &tp);

        if ((time->tv_nsec - tp.tv_nsec) < 0)
        {
            nsecond = (long)NANOSECOND_PER_SECOND - (tp.tv_nsec - time->tv_nsec);
            second  = time->tv_sec - tp.tv_sec - 1;
        }
        else
        {
            nsecond = time->tv_nsec - tp.tv_nsec;
            second  = time->tv_sec - tp.tv_sec;
        }

        /*
        * Warning: NANOSECOND_PER_SECOND is unsigned long, division method instruction will be `divu`.
        *          so then result is overflow undefined behavior.
        */
        tick = (int)(second * RT_TICK_PER_SECOND + (long)(nsecond * RT_TICK_PER_SECOND) / (long)NANOSECOND_PER_SECOND);
        if (tick < 0) tick = 0;
    }

    return tick;
}
RTM_EXPORT(clock_time_to_tick);

int clock_getres(clockid_t clockid, struct timespec *res)
{
    int ret = 0;

    if (res == RT_NULL)
    {
        rt_set_errno(EINVAL);
        return -1;
    }

    switch (clockid)
    {
    case CLOCK_REALTIME:
        res->tv_sec = 0;
        res->tv_nsec = NANOSECOND_PER_SECOND/RT_TICK_PER_SECOND;
        break;

#ifdef RT_USING_CPUTIME
    case CLOCK_MONOTONIC:
    case CLOCK_CPUTIME_ID:
        res->tv_sec  = 0;
        res->tv_nsec = clock_cpu_getres();
        break;
#endif

    default:
        ret = -1;
        rt_set_errno(EINVAL);
        break;
    }

    return ret;
}
RTM_EXPORT(clock_getres);

int clock_gettime(clockid_t clockid, struct timespec *tp)
{
    int ret = 0;

    if (tp == RT_NULL)
    {
        rt_set_errno(EINVAL);
        return -1;
    }

    switch (clockid)
    {
    case CLOCK_REALTIME:
        {
            /* get tick */
            rt_tick_t tick = rt_tick_get();

            tp->tv_sec  = _timevalue.tv_sec + tick / RT_TICK_PER_SECOND;
            tp->tv_nsec = (_timevalue.tv_usec + (tick % RT_TICK_PER_SECOND) * MICROSECOND_PER_TICK) * 1000;
        }
        break;

#ifdef RT_USING_CPUTIME
    case CLOCK_MONOTONIC:
    case CLOCK_CPUTIME_ID:
        {
            float unit = 0;
            uint64_t cpu_tick;

            unit = clock_cpu_getres();
            cpu_tick = clock_cpu_gettime();

            tp->tv_sec = ((uint64_t)(cpu_tick * unit)) / NANOSECOND_PER_SECOND;
            tp->tv_nsec = ((uint64_t)(cpu_tick * unit)) % NANOSECOND_PER_SECOND;
        }
        break;
#endif
    default:
        rt_set_errno(EINVAL);
        ret = -1;
    }

    return ret;
}
RTM_EXPORT(clock_gettime);

int clock_settime(clockid_t clockid, const struct timespec *tp)
{
    int second;
    rt_tick_t tick;
    rt_device_t device;

    if ((clockid != CLOCK_REALTIME) || (tp == RT_NULL))
    {
        rt_set_errno(EINVAL);

        return -1;
    }

    /* get second */
    second = tp->tv_sec;
    /* get tick */
    tick = rt_tick_get();

    /* update timevalue */
    _timevalue.tv_usec = MICROSECOND_PER_SECOND - (tick % RT_TICK_PER_SECOND) * MICROSECOND_PER_TICK;
    _timevalue.tv_sec = second - tick/RT_TICK_PER_SECOND - 1;

    /* update for RTC device */
    device = rt_device_find("rtc");
    if (device != RT_NULL)
    {
        /* set realtime seconds */
        rt_device_control(device, RT_DEVICE_CTRL_RTC_SET_TIME, &second);
    }
    else
        return -1;

    return 0;
}
RTM_EXPORT(clock_settime);

int clock_nanosleep(clockid_t clockid, int flags, const struct timespec *rqtp, struct timespec *rmtp)
{
    if (rqtp->tv_sec < 0 || rqtp->tv_nsec < 0 || rqtp->tv_nsec >= 1000000000)
    {
        rt_set_errno(EINVAL);
        return -1;
    }
    switch (clockid)
    {
    case CLOCK_REALTIME:
    {
        rt_tick_t tick;
        if (flags & TIMER_ABSTIME == TIMER_ABSTIME)
        {
            tick = (rqtp->tv_sec - _timevalue.tv_sec) * RT_TICK_PER_SECOND + ((uint64_t)(rqtp->tv_nsec - _timevalue.tv_usec) * RT_TICK_PER_SECOND) / 1000000000;
            rt_tick_t rt_tick = rt_tick_get();
            tick = tick < rt_tick ? 0 : tick - rt_tick;
        }
        else
        {
            tick = rqtp->tv_sec * RT_TICK_PER_SECOND + ((uint64_t)(rqtp->tv_nsec) * RT_TICK_PER_SECOND) / 1000000000;
        }
        rt_thread_delay(tick);
        if (rmtp)
        {
            tick = rt_tick_get() - tick;
            rmtp->tv_sec = tick / RT_TICK_PER_SECOND;
            rmtp->tv_nsec = (tick % RT_TICK_PER_SECOND) * (1000000000 / RT_TICK_PER_SECOND);
        }
    }
    break;

#ifdef RT_USING_CPUTIME
    case CLOCK_MONOTONIC:
    case CLOCK_CPUTIME_ID:
    {
        uint64_t cpu_tick, cpu_tick_old;
        cpu_tick_old = clock_cpu_gettime();
        rt_tick_t tick;
        float unit = clock_cpu_getres();

        cpu_tick = (rqtp->tv_sec * NANOSECOND_PER_SECOND + ((uint64_t)rqtp->tv_nsec * NANOSECOND_PER_SECOND) / 1000000000) / unit;
        if (flags & TIMER_ABSTIME == TIMER_ABSTIME)
            cpu_tick = cpu_tick < cpu_tick_old ? 0 : cpu_tick - cpu_tick_old;
        tick = cpu_tick / (NANOSECOND_PER_SECOND / RT_TICK_PER_SECOND);
        rt_thread_delay(tick);

        if (rmtp)
        {
            uint64_t rmtp_cpu_tick = clock_cpu_gettime() - tick * (NANOSECOND_PER_SECOND / RT_TICK_PER_SECOND);
            rmtp->tv_sec = ((int)(rmtp_cpu_tick * unit)) / NANOSECOND_PER_SECOND;
            rmtp->tv_nsec = ((int)(rmtp_cpu_tick * unit)) % NANOSECOND_PER_SECOND;
        }

        while (clock_cpu_gettime() - cpu_tick_old < cpu_tick);
    }
    break;
#endif
    default:
        rt_set_errno(EINVAL);
        return -1;
    }

    return 0;
}
RTM_EXPORT(clock_nanosleep);

int nanosleep(const struct timespec *rqtp, struct timespec *rmtp)
{
    if (rqtp->tv_sec < 0 || rqtp->tv_nsec < 0 || rqtp->tv_nsec >= 1000000000)
    {
        rt_set_errno(EINVAL);
        return -1;
    }
#ifdef RT_USING_CPUTIME
    uint64_t cpu_tick, cpu_tick_old;
    cpu_tick_old = clock_cpu_gettime();
    rt_tick_t tick;
    float unit = clock_cpu_getres();

    cpu_tick = (rqtp->tv_sec * NANOSECOND_PER_SECOND + ((uint64_t)rqtp->tv_nsec * NANOSECOND_PER_SECOND) / 1000000000)/unit;
    tick = cpu_tick  / (NANOSECOND_PER_SECOND / RT_TICK_PER_SECOND);
    rt_thread_delay(tick);

    if (rmtp)
    {
        uint64_t rmtp_cpu_tick = clock_cpu_gettime() - tick * (NANOSECOND_PER_SECOND / RT_TICK_PER_SECOND);
        rmtp->tv_sec = ((int)(rmtp_cpu_tick * unit)) / NANOSECOND_PER_SECOND;
        rmtp->tv_nsec = ((int)(rmtp_cpu_tick * unit)) % NANOSECOND_PER_SECOND;
    }

    while (clock_cpu_gettime() - cpu_tick_old < cpu_tick);
#else
    rt_tick_t tick;
    tick = rqtp->tv_sec * RT_TICK_PER_SECOND + ((uint64_t)rqtp->tv_nsec * RT_TICK_PER_SECOND) / 1000000000;
    rt_thread_delay(tick);

    if (rmtp)
    {
        tick = rt_tick_get() - tick;
        /* get the passed time */
        rmtp->tv_sec = tick / RT_TICK_PER_SECOND;
        rmtp->tv_nsec = (tick % RT_TICK_PER_SECOND) * (1000000000 / RT_TICK_PER_SECOND);
    }
#endif
    return 0;
}
RTM_EXPORT(nanosleep);
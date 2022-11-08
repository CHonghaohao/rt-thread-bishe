/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-02-03     lizhirui     first version
 */

#include <rthw.h>
#include <rtthread.h>

#define DBG_TAG "syscall"
#define DBG_LVL DBG_WARNING
#include <rtdbg.h>

#include <stdint.h>
#include <mmu.h>
#include <page.h>
#include <lwp_mm_area.h>
#include <lwp_user_mm.h>

#include <stdio.h>

#include "riscv_mmu.h"
#include "stack.h"

typedef rt_size_t (*syscallfunc_t)(rt_size_t, rt_size_t, rt_size_t, rt_size_t, rt_size_t, rt_size_t, rt_size_t);
syscallfunc_t lwp_get_sys_api(uint32_t);

static char *syscall_name[256];

void syscall_handler(struct rt_hw_stack_frame *regs)
{
    int syscallid = regs->a7;
    if (syscallid == 0)
    {
        LOG_E("syscall id = 0!\n");
        while (1)
            ;
    }

    syscallfunc_t syscallfunc = (syscallfunc_t)lwp_get_sys_api(syscallid);

    if (syscallfunc == RT_NULL)
    {
        LOG_E("unsupported syscall!\n");
        sys_exit(-1);
    }

    LOG_I("[0x%lx] %s(0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx)", rt_thread_self(), syscall_name[syscallid],
        regs->a0, regs->a1, regs->a2, regs->a3, regs->a4, regs->a5, regs->a6);
    regs->a0 = syscallfunc(regs->a0, regs->a1, regs->a2, regs->a3, regs->a4, regs->a5, regs->a6);
    regs->a7 = 0;
    regs->epc += 4; // skip ecall instruction
    LOG_I("[0x%lx] %s ret: 0x%lx", rt_thread_self(), syscall_name[syscallid], regs->a0);
}

static char *syscall_name[] = {
    "UNDEFINED",
    "sys_exit",
    "sys_read",
    "sys_write",
    "sys_lseek",
    "sys_open",
    "sys_close",
    "sys_ioctl",
    "sys_fstat",
    "sys_poll",
    "sys_nanosleep",
    "sys_gettimeofday",
    "sys_settimeofday",
    "sys_exec",
    "sys_kill",
    "sys_getpid",
    "sys_getpriority",
    "sys_setpriority",
    "sys_sem_create",
    "sys_sem_delete",
    "sys_sem_take",
    "sys_sem_release",
    "sys_mutex_create",
    "sys_mutex_delete",
    "sys_mutex_take",
    "sys_mutex_release",
    "sys_event_create",
    "sys_event_delete",
    "sys_event_send",
    "sys_event_recv",
    "sys_mb_create",
    "sys_mb_delete",
    "sys_mb_send",
    "sys_mb_send_wait",
    "sys_mb_recv",
    "sys_mq_create",
    "sys_mq_delete",
    "sys_mq_send",
    "sys_mq_urgent",
    "sys_mq_recv",
    "sys_thread_create",
    "sys_thread_delete",
    "sys_thread_startup",
    "sys_thread_self",
    "sys_channel_open",
    "sys_channel_close",
    "sys_channel_send",
    "sys_channel_send_recv_timeout",
    "sys_channel_reply",
    "sys_channel_recv_timeout",
    "sys_enter_critical",
    "sys_exit_critical",
    "sys_brk",
    "sys_mmap2",
    "sys_munmap",
    "sys_shmget",
    "sys_shmrm",
    "sys_shmat",
    "sys_shmdt",
    "sys_device_init",
    "sys_device_register",
    "sys_device_control",
    "sys_device_find",
    "sys_device_open",
    "sys_device_close",
    "sys_device_read",
    "sys_device_write",
    "sys_stat",
    "sys_thread_find",
    "sys_accept",
    "sys_bind",
    "sys_shutdown",
    "sys_getpeername",
    "sys_getsockname",
    "sys_getsockopt",
    "sys_setsockopt",
    "sys_connect",
    "sys_listen",
    "sys_recv",
    "sys_recvfrom",
    "sys_send",
    "sys_sendto",
    "sys_socket",
    "sys_closesocket",
    "sys_getaddrinfo",
    "sys_gethostbyname2_r",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_select",
    "sys_notimpl",
    "sys_notimpl",
    "sys_tick_get",
    "sys_exit_group",
    "sys_notimpl",
    "sys_notimpl",
    "sys_notimpl",
    "sys_thread_mdelay",
    "sys_sigaction",
    "sys_sigprocmask",
    "sys_tkill",
    "sys_thread_sigprocmask",
    "sys_cacheflush",
    "sys_notimpl",
    "sys_notimpl",
    "sys_waitpid",
    "sys_timer_create",
    "sys_timer_delete",
    "sys_timer_start",
    "sys_timer_stop",
    "sys_timer_control",
    "sys_getcwd",
    "sys_chdir",
    "sys_unlink",
    "sys_mkdir",
    "sys_rmdir",
    "sys_getdents",
    "sys_get_errno",
    "sys_set_thread_area",
    "sys_set_tid_address",
    "sys_access",
    "sys_pipe",
    "sys_clock_settime",
    "sys_clock_gettime",
    "sys_clock_getres",
    "sys_clone",
    "sys_futex",
    "sys_pmutex",
    "sys_dup",
    "sys_dup2",
    "sys_rename",
    "sys_fork",
    "sys_execve",
    "sys_vfork",
    "sys_gettid",
    "sys_prlimit64",
    "sys_getrlimit",
    "sys_setrlimit",
    "sys_setsid",
    "sys_getrandom",
    "sys_notimpl",
    "sys_mremap",
    "sys_madvise",
    "sys_sched_setparam",
    "sys_sched_getparam",
    "sys_sched_get_priority_max",
    "sys_sched_get_priority_min",
    "sys_sched_setscheduler",
    "sys_sched_getscheduler",
    "sys_setaffinity",
    "sys_fsync",
    [255] = "sys_log"
};

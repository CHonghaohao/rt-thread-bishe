CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_PROCESSOR cortex-a)
#SET(CMAKE_VERBOSE_MAKEFILE ON)

SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_C_COMPILER "/home/rsl/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/aarch64-none-elf-gcc")
SET(CMAKE_ASM_COMPILER "/home/rsl/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/aarch64-none-elf-gcc")
SET(CMAKE_C_FLAGS " -g -march=armv8-a -mtune=cortex-a55 -Wall -Wno-cpp -O0 -gdwarf-2")
SET(CMAKE_ASM_FLAGS " -c -x assembler-with-cpp -D__ASSEMBLY__ -gdwarf-2")
SET(CMAKE_C_COMPILER_WORKS TRUE)

SET(CMAKE_CXX_COMPILER "/home/rsl/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/aarch64-none-elf-g++")
SET(CMAKE_CXX_FLAGS " -g -march=armv8-a -mtune=cortex-a55 -Wall -Wno-cpp -O0 -gdwarf-2")
SET(CMAKE_CXX_COMPILER_WORKS TRUE)

SET(CMAKE_OBJCOPY "/home/rsl/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/aarch64-none-elf-objcopy")
SET(CMAKE_SIZE "/home/rsl/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/aarch64-none-elf-size")

SET(CMAKE_EXE_LINKER_FLAGS " -g -march=armv8-a -mtune=cortex-a55 -nostartfiles -Wl,--gc-sections,-Map=rtthread.map,-cref,-u,system_vectors -T ${CMAKE_SOURCE_DIR}/link.lds -Wl,--gc-sections,--print-memory-usage -nostartfiles")

SET(CMAKE_C_STANDARD 11)
SET(CMAKE_CXX_STANDARD 17)

PROJECT(rtthread C CXX ASM)

INCLUDE_DIRECTORIES(
	.
	packages/micro_ros_rtthread_component/builder/libmicroros/include
	packages/micro_ros_rtthread_component/include
	packages/micro_ros_rtthread_component/examples
	applications
	driver
	driver/CMSIS
	../../../include
	../../../libcpu/aarch64/common
	../../../libcpu/aarch64/common/include
	../../../libcpu/aarch64/cortex-a
	../../../components/utilities/libadt/hashmap
	../../../components/utilities/libadt/bitmap
	../../../components/utilities/libadt/avl
	../../../components/utilities/libadt/uthash
	../../../components/utilities/libadt/ref
	../../../components/legacy/usb/usbhost
	../../../components/legacy/usb/usbhost/class
	../../../components/legacy/usb/usbhost/core
	../../../components/legacy/usb/usbhost/include
	../../../components/legacy/include
	../../../components/libc/compilers/newlib
	../../../components/libc/compilers/common/include
	../../../components/libc/posix/io/eventfd
	../../../components/libc/posix/io/poll
	../../../components/libc/posix/io/epoll
	../../../components/libc/posix/ipc
	../../../components/libc/posix/delay
	../../../components/net/netdev/include
	../../../components/net/sal/include
	../../../components/net/sal/include/socket
	../../../components/net/sal/include/dfs_net
	../../../components/net/sal/include/socket/sys_socket
	../../../components/drivers/include
	../../../components/drivers/ktime
	../../../components/drivers/ktime/inc
	../../../components/drivers/phy
	../../../components/drivers/smp_call
	../../../components/drivers/ofw
	../../../components/drivers/ofw/libfdt
	../../../components/dfs/dfs_v1/include
	../../../components/dfs/dfs_v1/filesystems/devfs
	../../../components/mm
	../../../components/finsh
)

ADD_DEFINITIONS(
	-D__RTTHREAD__
	-DRT_USING_NEWLIBC
	-DRT_USING_LIBC
	-D_POSIX_C_SOURCE=1
)

# Library source files
SET(RT_APPLICATIONS_SOURCES
	applications/ros_test.c
	applications/main.c
)

SET(RT_COMPILER_SOURCES
	../../../components/libc/compilers/common/cctype.c
	../../../components/libc/compilers/common/cstdlib.c
	../../../components/libc/compilers/common/cstring.c
	../../../components/libc/compilers/common/ctime.c
	../../../components/libc/compilers/common/cunistd.c
	../../../components/libc/compilers/common/cwchar.c
	../../../components/libc/compilers/newlib/syscalls.c
)

SET(RT_DEVICEDRIVERS_SOURCES
	../../../components/drivers/clk/clk-fixed-rate.c
	../../../components/drivers/clk/clk.c
	../../../components/drivers/core/bus.c
	../../../components/drivers/core/device.c
	../../../components/drivers/core/dm.c
	../../../components/drivers/core/driver.c
	../../../components/drivers/core/mnt.c
	../../../components/drivers/core/numa.c
	../../../components/drivers/core/platform.c
	../../../components/drivers/core/platform_ofw.c
	../../../components/drivers/core/power_domain.c
	../../../components/drivers/iio/iio.c
	../../../components/drivers/ipc/completion_comm.c
	../../../components/drivers/ipc/completion_mp.c
	../../../components/drivers/ipc/condvar.c
	../../../components/drivers/ipc/dataqueue.c
	../../../components/drivers/ipc/pipe.c
	../../../components/drivers/ipc/ringblk_buf.c
	../../../components/drivers/ipc/ringbuffer.c
	../../../components/drivers/ipc/waitqueue.c
	../../../components/drivers/ipc/workqueue.c
	../../../components/drivers/ofw/base.c
	../../../components/drivers/ofw/fdt.c
	../../../components/drivers/ofw/io.c
	../../../components/drivers/ofw/libfdt/fdt.c
	../../../components/drivers/ofw/libfdt/fdt_addresses.c
	../../../components/drivers/ofw/libfdt/fdt_empty_tree.c
	../../../components/drivers/ofw/libfdt/fdt_overlay.c
	../../../components/drivers/ofw/libfdt/fdt_ro.c
	../../../components/drivers/ofw/libfdt/fdt_rw.c
	../../../components/drivers/ofw/libfdt/fdt_strerror.c
	../../../components/drivers/ofw/libfdt/fdt_sw.c
	../../../components/drivers/ofw/libfdt/fdt_wip.c
	../../../components/drivers/ofw/ofw.c
	../../../components/drivers/ofw/raw.c
	../../../components/drivers/pin/dev_pin.c
	../../../components/drivers/pin/dev_pin_dm.c
	../../../components/drivers/pin/dev_pin_ofw.c
	../../../components/drivers/pm/lptimer.c
	../../../components/drivers/pm/pm.c
	../../../components/drivers/serial/dev_serial.c
	../../../components/drivers/serial/serial_dm.c
)

SET(RT_DRIVERS_SOURCES
	driver/CMSIS/irq_ctrl_gic.c
	driver/board.c
	driver/drv_uart.c
	driver/hal_canfd.c
	driver/hal_gmac.c
	driver/hal_gmac_rk3568.c
	driver/hal_pinctrl_rk3588.c
	driver/system_rk3588.c
)

SET(RT_FILESYSTEM_SOURCES
	../../../components/dfs/dfs_v1/filesystems/devfs/devfs.c
	../../../components/dfs/dfs_v1/src/dfs.c
	../../../components/dfs/dfs_v1/src/dfs_file.c
	../../../components/dfs/dfs_v1/src/dfs_fs.c
	../../../components/dfs/dfs_v1/src/dfs_posix.c
)

SET(RT_FINSH_SOURCES
	../../../components/finsh/msh.c
	../../../components/finsh/msh_parse.c
	../../../components/finsh/shell.c
	../../../components/finsh/msh_file.c
	../../../components/finsh/cmd.c
)

SET(RT_KERNEL_SOURCES
	../../../src/clock.c
	../../../src/components.c
	../../../src/cpu_mp.c
	../../../src/defunct.c
	../../../src/idle.c
	../../../src/ipc.c
	../../../src/irq.c
	../../../src/kservice.c
	../../../src/mem.c
	../../../src/memheap.c
	../../../src/mempool.c
	../../../src/object.c
	../../../src/scheduler_comm.c
	../../../src/scheduler_mp.c
	../../../src/thread.c
	../../../src/timer.c
)

SET(RT_KLIBC_SOURCES
	../../../src/klibc/rt_vsnprintf_std.c
	../../../src/klibc/kstdio.c
	../../../src/klibc/rt_vsscanf.c
	../../../src/klibc/kerrno.c
	../../../src/klibc/kstring.c
)

SET(RT_KTIME_SOURCES
	../../../components/drivers/ktime/src/aarch64/cputimer.c
	../../../components/drivers/ktime/src/cputimer.c
	../../../components/drivers/ktime/src/boottime.c
	../../../components/drivers/ktime/src/hrtimer.c
)

SET(RT_LIBADT_SOURCES
	../../../components/utilities/libadt/avl/avl.c
)

SET(RT_LIBCPU_SOURCES
	../../../libcpu/aarch64/common/atomic_aarch64.c
	../../../libcpu/aarch64/common/backtrace.c
	../../../libcpu/aarch64/common/cache.S
	../../../libcpu/aarch64/common/cache_ops.c
	../../../libcpu/aarch64/common/cpu.c
	../../../libcpu/aarch64/common/cpu_gcc.S
	../../../libcpu/aarch64/common/cpu_psci.c
	../../../libcpu/aarch64/common/cpu_spin_table.c
	../../../libcpu/aarch64/common/cpuport.c
	../../../libcpu/aarch64/common/exception.c
	../../../libcpu/aarch64/common/gic.c
	../../../libcpu/aarch64/common/gicv3.c
	../../../libcpu/aarch64/common/gtimer.c
	../../../libcpu/aarch64/common/hypercall.c
	../../../libcpu/aarch64/common/interrupt.c
	../../../libcpu/aarch64/common/mmu.c
	../../../libcpu/aarch64/common/mp/context_gcc.S
	../../../libcpu/aarch64/common/mp/vector_gcc.S
	../../../libcpu/aarch64/common/psci.c
	../../../libcpu/aarch64/common/setup.c
	../../../libcpu/aarch64/common/smccc.S
	../../../libcpu/aarch64/common/stack.c
	../../../libcpu/aarch64/common/stack_gcc.S
	../../../libcpu/aarch64/common/startup_gcc.S
	../../../libcpu/aarch64/common/trap.c
	../../../libcpu/aarch64/common/vector_gcc.S
	../../../libcpu/aarch64/cortex-a/entry_point.S
)

SET(RT_MICROROS_SOURCES
	packages/micro_ros_rtthread_component/src/rtt_serial_transport.c
)

SET(RT_MM_SOURCES
	../../../components/mm/mm_memblock.c
	../../../components/mm/mm_anon.c
	../../../components/mm/mm_aspace.c
	../../../components/mm/mm_object.c
	../../../components/mm/avl_adpt.c
	../../../components/mm/mm_kmem.c
	../../../components/mm/mm_page.c
	../../../components/mm/ioremap.c
	../../../components/mm/mm_fault.c
)

SET(RT_POSIX_SOURCES
	../../../components/libc/posix/delay/delay.c
	../../../components/libc/posix/io/poll/poll.c
	../../../components/libc/posix/io/poll/select.c
)

SET(RT_RT_USBH_SOURCES
	../../../components/legacy/usb/usbhost/core/driver.c
	../../../components/legacy/usb/usbhost/core/usbhost_core.c
	../../../components/legacy/usb/usbhost/core/hub.c
	../../../components/legacy/usb/usbhost/core/usbhost.c
)

SET(RT_SAL_SOURCES
	../../../components/net/netdev/src/netdev.c
	../../../components/net/netdev/src/netdev_ipaddr.c
	../../../components/net/sal/dfs_net/dfs_net.c
	../../../components/net/sal/socket/net_netdb.c
	../../../components/net/sal/socket/net_sockets.c
	../../../components/net/sal/src/sal_socket.c
)

SET(RT_SMP_SOURCES
	../../../components/drivers/smp_call/smp_call.c
)

SET(RT_UTESTCASES_SOURCES
)

# Library search paths
SET(RT_MICROROS_LINK_DIRS
	/home/rsl/rt-thread/bsp/rockchip/rk3568/packages/micro_ros_rtthread_component/builder/libmicroros
)

# Library local macro definitions
SET(RT_DEVICEDRIVERS_DEFINES
	__RT_IPC_SOURCE__
)

SET(RT_KERNEL_DEFINES
	__RT_KERNEL_SOURCE__
)

# Library dependencies
SET(RT_COMPILER_LIBS
	c
	m
)

SET(RT_MICROROS_LIBS
	microros
)

# Libraries
ADD_LIBRARY(rtt_Compiler OBJECT ${RT_COMPILER_SOURCES})
ADD_LIBRARY(rtt_DeviceDrivers OBJECT ${RT_DEVICEDRIVERS_SOURCES})
ADD_LIBRARY(rtt_Drivers OBJECT ${RT_DRIVERS_SOURCES})
ADD_LIBRARY(rtt_Filesystem OBJECT ${RT_FILESYSTEM_SOURCES})
ADD_LIBRARY(rtt_Finsh OBJECT ${RT_FINSH_SOURCES})
ADD_LIBRARY(rtt_Kernel OBJECT ${RT_KERNEL_SOURCES})
ADD_LIBRARY(rtt_klibc OBJECT ${RT_KLIBC_SOURCES})
ADD_LIBRARY(rtt_ktime OBJECT ${RT_KTIME_SOURCES})
ADD_LIBRARY(rtt_LIBADT OBJECT ${RT_LIBADT_SOURCES})
ADD_LIBRARY(rtt_libcpu OBJECT ${RT_LIBCPU_SOURCES})
ADD_LIBRARY(rtt_microros OBJECT ${RT_MICROROS_SOURCES})
ADD_LIBRARY(rtt_mm OBJECT ${RT_MM_SOURCES})
ADD_LIBRARY(rtt_POSIX OBJECT ${RT_POSIX_SOURCES})
ADD_LIBRARY(rtt_rt_usbh OBJECT ${RT_RT_USBH_SOURCES})
ADD_LIBRARY(rtt_SAL OBJECT ${RT_SAL_SOURCES})
ADD_LIBRARY(rtt_smp OBJECT ${RT_SMP_SOURCES})

# Interface libraries
ADD_LIBRARY(rtt_utestcases INTERFACE)

# Private macros
TARGET_COMPILE_DEFINITIONS(rtt_DeviceDrivers PRIVATE ${RT_DEVICEDRIVERS_DEFINES})
TARGET_COMPILE_DEFINITIONS(rtt_Kernel PRIVATE ${RT_KERNEL_DEFINES})

# Interface library search paths
TARGET_LINK_DIRECTORIES(rtt_microros INTERFACE ${RT_MICROROS_LINK_DIRS})
TARGET_LINK_LIBRARIES(rtt_Compiler INTERFACE ${RT_COMPILER_LIBS})
TARGET_LINK_LIBRARIES(rtt_microros INTERFACE ${RT_MICROROS_LIBS})

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME}.elf ${RT_APPLICATIONS_SOURCES})
TARGET_LINK_LIBRARIES(${CMAKE_PROJECT_NAME}.elf
	rtt_Compiler
	rtt_DeviceDrivers
	rtt_Drivers
	rtt_Filesystem
	rtt_Finsh
	rtt_Kernel
	rtt_klibc
	rtt_ktime
	rtt_LIBADT
	rtt_libcpu
	rtt_microros
	rtt_mm
	rtt_POSIX
	rtt_rt_usbh
	rtt_SAL
	rtt_smp
	rtt_utestcases
)

ADD_CUSTOM_COMMAND(TARGET ${CMAKE_PROJECT_NAME}.elf POST_BUILD 
	COMMAND ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME}.elf rtthread.bin
	COMMAND ${CMAKE_SIZE} ${CMAKE_PROJECT_NAME}.elf
)

# if custom.cmake is exist, add it
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake)
endif()

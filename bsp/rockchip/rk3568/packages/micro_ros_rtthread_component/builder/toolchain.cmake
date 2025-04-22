include(CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-a)

set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TARGET "/home/rsl/gcc-arm-10.3-2021.07-x86_64-aarch64-none-elf/bin/")
set(CMAKE_C_COMPILER "${TARGET}aarch64-none-elf-gcc")
set(CMAKE_CXX_COMPILER "${TARGET}aarch64-none-elf-g++")
set(CMAKE_AR "${TARGET}aarch64-none-elf-ar")

set(FLAGS_AARCH64 
    "-O2 -mcpu=cortex-a55+simd -ffunction-sections -fdata-sections -nostdlib --param max-inline-insns-single=500  -fno-exceptions -mlittle-endian -mcmodel=small ")

set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS_AARCH64} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='")
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS_AARCH64} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='")

set(__BIG_ENDIAN__ 0)

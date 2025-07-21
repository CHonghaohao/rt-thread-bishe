# RK3588 BSP Introduction 

## 编译环境需求

下载编译工具链接：https://developer.arm.com/downloads/-/gnu-a/10-3-2021-07

配置编译工具的位置：./rtconfig.py -> EXEC_PATH

## 移植RK3588

RK3588由RK3568的工程移植而来，且目前放置了32位rtthread的较全的HAL库，项目所需的canfd、gmac驱动源码和microros源码。
系统可由裸机启动，采用串口2，波特率为1500000.
multicore/rpmsg_lite/readme.txt

The RPMsg Lite component is a lightweight implementation of the RPMsg protocol.
The RPMsg protocol defines a standardized binary interface used to communicate between
multiple cores in a heterogeneous multicore system.

Directory Structure

doc - Holds the documentation.
lib - Holds source code for rpmsg_lite.

Manual enable without menuconfig
-------------------------------
If you skip `scons --menuconfig`, you can still enable the rpmsg-lite
package by adding the following switches to your BSP's `rtconfig.h`:

```
/* Enable rpmsg-lite package build */
#define PKG_USING_RPMSG_LITE

/* Keep the default rpmsg-lite options (optional) */
#define PKG_USING_RPMSG_LITE_DEFAULT
```

The `PKG_USING_RPMSG_LITE` macro is the gate used by
`bsp/rockchip/rk3588/packages/rpmsg_lite/SConscript` to pull the sources
into the build. `PKG_USING_RPMSG_LITE_DEFAULT` can be left commented out
or undefined if you want to override rpmsg-lite defaults manually.

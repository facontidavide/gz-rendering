Patches applied on top of Filament 0643fd0f for this spike:

- 0001: build with GCC (-fstrict-vtable-pointers is clang only) and compile
  the built-in materials for all platforms when EGL is used on Linux (EGL
  selects the GLES backend, which needs the mobile shader variants).
- 0002: Vulkan readPixels reuses staging buffers instead of allocating
  device memory per readback (upstream TODO in VulkanReadPixels.cpp). Cuts
  the Vulkan depth camera from 1.47 to 0.96 ms per frame here.
- 0003 (on top of 0002): Vulkan readPixels also recycles its fences and
  command buffers instead of creating/destroying them per readback (driver
  ioctls on NVIDIA), keeps staging memory mapped for its whole life, and
  DataReshaper memcpys rows when source and destination layouts match (same
  type and channel count, no swizzle, zero offset) for UBYTE/FLOAT/INT/UINT,
  not only RGBA8. Output bytes unchanged.

Apply with `git apply` in the Filament checkout, then configure with
-DFILAMENT_ENABLE_EXPERIMENTAL_GCC_SUPPORT=ON -DFILAMENT_SUPPORTS_EGL_ON_LINUX=ON
(see ../tools/gzsim/build_in_container.sh for the full command line).

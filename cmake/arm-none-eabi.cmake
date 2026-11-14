# Cross toolchain for the HJ-20 controller (STM32G474RE, Cortex-M4F).
#
#   cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#
# Bare metal, no newlib: the image links with -nostdlib, so nothing GPL or
# third-party ends up in the binary (see sbom/components.json).

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE FILEPATH "objcopy")
set(CMAKE_SIZE arm-none-eabi-size CACHE FILEPATH "size")

set(HJ20_MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT
    "${HJ20_MCU_FLAGS} -ffreestanding -fno-math-errno -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_C_FLAGS_RELEASE_INIT "-O2 -g3")
set(CMAKE_C_FLAGS_DEBUG_INIT "-Og -g3")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${HJ20_MCU_FLAGS} -nostdlib -nostartfiles -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

cmake_minimum_required(VERSION 3.20)

project(hj20_firmware
    VERSION 0.2.2
    DESCRIPTION "HJ-20 cobot wrist joint actuator firmware"
    LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(HJ20_WERROR "Treat compiler warnings as errors" OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Portable control core: no HAL, no heap, no libc beyond <stdint.h>/<stdbool.h>
# ---------------------------------------------------------------------------
add_library(hj20_core STATIC
    src/fastmath.c
    src/foc.c
    src/pi.c
    src/motion.c
    src/torque.c
    src/encoder.c
    src/fault.c
    src/can_proto.c
    src/control.c
)
target_include_directories(hj20_core PUBLIC src)
target_compile_definitions(hj20_core PUBLIC
    HJ20_FW_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    HJ20_FW_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    HJ20_FW_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)
target_compile_options(hj20_core PRIVATE
    -Wall -Wextra -Wshadow -Wconversion -Wdouble-promotion -Wstrict-prototypes
    $<$<BOOL:${HJ20_WERROR}>:-Werror>
)

if(CMAKE_CROSSCOMPILING)
    # Target firmware image for the STM32G474RE controller board.
    add_subdirectory(bsp/stm32g474)
else()
    # Host build: unit tests with JUnit XML output.
    enable_testing()
    add_subdirectory(tests)
endif()

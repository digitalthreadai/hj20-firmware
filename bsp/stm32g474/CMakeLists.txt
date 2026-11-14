set(HJ20_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/stm32g474re.ld)

add_executable(hj20_firmware
    startup.c
    libc_min.c
    board.c
    drv8353.c
    main.c
)
set_target_properties(hj20_firmware PROPERTIES
    OUTPUT_NAME hj20-firmware
    SUFFIX .elf
    LINK_DEPENDS ${HJ20_LINKER_SCRIPT}
)
target_link_libraries(hj20_firmware PRIVATE hj20_core)
target_compile_options(hj20_firmware PRIVATE -Wall -Wextra)

# memcpy/memset must not be "optimised" into calls to themselves.
set_source_files_properties(libc_min.c PROPERTIES
    COMPILE_OPTIONS "-fno-tree-loop-distribute-patterns")

target_link_options(hj20_firmware PRIVATE
    -T${HJ20_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/hj20-firmware.map
    -Wl,--print-memory-usage
)

add_custom_command(TARGET hj20_firmware POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:hj20_firmware>
            ${CMAKE_CURRENT_BINARY_DIR}/hj20-firmware.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:hj20_firmware>
    COMMENT "objcopy -> hj20-firmware.bin"
    VERBATIM
)

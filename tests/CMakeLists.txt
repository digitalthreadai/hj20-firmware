add_executable(hj20_tests
    test_main.c
    test_foc.c
    test_pi.c
    test_torque.c
)
target_link_libraries(hj20_tests PRIVATE hj20_core m)
target_include_directories(hj20_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(hj20_tests PRIVATE -Wall -Wextra $<$<BOOL:${HJ20_WERROR}>:-Werror>)

# One test entry, one JUnit file: the CI artifact `junit-results` uploads
# exactly this path (the PLM ingests the first .xml in that artifact).
set(HJ20_JUNIT_DIR ${CMAKE_BINARY_DIR}/test-results)
file(MAKE_DIRECTORY ${HJ20_JUNIT_DIR})
add_test(NAME hj20_unit_tests
         COMMAND hj20_tests --junit ${HJ20_JUNIT_DIR}/junit.xml)

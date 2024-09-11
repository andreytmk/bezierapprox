cmake_minimum_required(VERSION 3.8)

project(bezierapproxlib VERSION 1.0.0 DESCRIPTION "Approximation with Bezier curves.")

include(GNUInstallDirs)

add_library(bezierapproxlib SHARED
    src/bezierapprox.c)

set_target_properties(bezierapproxlib PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    PUBLIC_HEADER include/bezierapprox.h)

configure_file(bezierapproxlib.pc.in bezierapproxlib.pc @ONLY)

target_include_directories(bezierapproxlib PRIVATE include)

target_compile_definitions(bezierapproxlib PRIVATE BEZIERAPPROXLIB_COMPILING=1)

find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(bezierapproxlib PUBLIC ${MATH_LIBRARY})
endif()

install(TARGETS bezierapproxlib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${CMAKE_BINARY_DIR}/bezierapproxlib.pc
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)

add_executable (bezierapprox_tests tests/bezierapprox_tests.c)
target_link_libraries (bezierapprox_tests bezierapproxlib)
target_include_directories(bezierapprox_tests PRIVATE include)

enable_testing()
add_test(TestBezierapproxlib bezierapprox_tests)

#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Derived from seL4test projects/sel4test/CMakeLists.txt and
# apps/sel4test-driver/CMakeLists.txt at
# 60b7b47ee0a67f56e15951735a6aa9d270d47f55.
#

cmake_minimum_required(VERSION 3.16.0)

include(settings.cmake)

project(zero_copy C ASM)

find_package(seL4 REQUIRED)
find_package(elfloader-tool REQUIRED)
find_package(musllibc REQUIRED)
find_package(util_libs REQUIRED)
find_package(seL4_libs REQUIRED)

sel4_import_kernel()
ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
elfloader_import_project()

musllibc_setup_build_environment_with_sel4runtime()
sel4_import_libsel4()
util_libs_import_libraries()
sel4_libs_import_libraries()

# The client runs in its own address space; the root task loads it from a
# CPIO archive linked into its image.
add_executable(zc_client EXCLUDE_FROM_ALL src/client.c)
target_link_libraries(zc_client PUBLIC sel4_autoconf muslc sel4 sel4runtime sel4muslcsys utils)
target_compile_options(zc_client PRIVATE -Wall -Werror)

include(cpio)
MakeCPIO(archive.o "$<TARGET_FILE:zc_client>")

add_executable(zero_copy src/manager.c archive.o)
target_link_libraries(
    zero_copy
    PUBLIC
        sel4_autoconf
        muslc
        sel4
        sel4runtime
        sel4allocman
        sel4vka
        sel4vspace
        sel4utils
        sel4simple
        sel4simple-default
        sel4platsupport
        sel4muslcsys
        platsupport
        utils
)
target_compile_options(zero_copy PRIVATE -Wall -Werror)

include(rootserver)
DeclareRootserver(zero_copy)

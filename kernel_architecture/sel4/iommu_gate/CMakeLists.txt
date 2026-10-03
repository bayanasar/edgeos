#
# Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Derived from seL4test projects/sel4test/CMakeLists.txt at
# 60b7b47ee0a67f56e15951735a6aa9d270d47f55.
#

cmake_minimum_required(VERSION 3.16.0)

include(settings.cmake)

project(iommu_gate C ASM)

find_package(seL4 REQUIRED)
find_package(elfloader-tool REQUIRED)
find_package(musllibc REQUIRED)
find_package(util_libs REQUIRED)
find_package(seL4_libs REQUIRED)

# Room for the ECAM and BAR frame capabilities.
set(KernelRootCNodeSizeBits 14 CACHE INTERNAL "")

sel4_import_kernel()
ApplyData61ElfLoaderSettings(${KernelPlatform} ${KernelSel4Arch})
elfloader_import_project()

musllibc_setup_build_environment_with_sel4runtime()
sel4_import_libsel4()
util_libs_import_libraries()
sel4_libs_import_libraries()

# The untrusted driver runs in its own address space; the root task, which
# owns the IOMMU, loads it from a CPIO archive linked into its image.
add_executable(gate_driver EXCLUDE_FROM_ALL src/driver.c src/edu.c)
target_link_libraries(gate_driver PUBLIC sel4_autoconf muslc sel4 sel4runtime sel4muslcsys utils)
target_compile_options(gate_driver PRIVATE -Wall -Werror)

include(cpio)
MakeCPIO(archive.o "$<TARGET_FILE:gate_driver>")

add_executable(iommu_gate src/main.c src/pci.c src/riscv_iommu.c src/edu.c archive.o)
target_link_libraries(
    iommu_gate
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
target_compile_options(iommu_gate PRIVATE -Wall -Werror)

include(rootserver)
DeclareRootserver(iommu_gate)

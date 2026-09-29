cmake_minimum_required(VERSION 3.16.0)

# Staged at <workspace>/projects/iommu_gate inside a pinned seL4test tree.
set(project_dir "${CMAKE_CURRENT_LIST_DIR}/../..")
file(GLOB project_modules ${project_dir}/projects/*)
list(APPEND CMAKE_MODULE_PATH ${project_dir}/kernel ${project_dir}/tools/seL4/cmake-tool/helpers/
     ${project_dir}/tools/seL4/elfloader-tool/ ${project_modules})
set(OPENSBI_PATH "${project_dir}/tools/opensbi" CACHE STRING "OpenSBI Folder location")
set(SEL4_CONFIG_DEFAULT_ADVANCED ON)
include(application_settings)

set(PLATFORM "qemu-riscv-virt" CACHE STRING "")
set(RISCV64 ON CACHE BOOL "")
correct_platform_strings()

find_package(seL4 REQUIRED)
sel4_configure_platform_settings()

set(SIMULATION ON CACHE BOOL "" FORCE)
ApplyCommonSimulationSettings(${KernelSel4Arch})
ApplyCommonReleaseVerificationSettings(FALSE FALSE)
set(KernelMaxNumNodes 1 CACHE STRING "" FORCE)
set(KernelIsMCS OFF CACHE BOOL "" FORCE)

# CLI/CI build of the ZeroPilot 4.0 STM32H753IITX application image.
#
# STM32CubeIDE's managed build (.cproject) still owns the H753 board files, so
# this reconstructs the link it performs from .cproject/.mxproject. It follows
# the CMakePresets.json + toolchain-file pattern from stm32l552xx (chore/cmake),
# but the STM32CubeIDE "Convert to CMake" tool has not been run on this target,
# so sources here are collected with file(GLOB) rather than hand-listed as a
# cmake/stm32cubemx export would be -- happy to swap in a real export if/when
# someone runs the CubeIDE conversion on this target.
cmake_minimum_required(VERSION 3.22)
project(stm32h753iitx C CXX ASM)

set(ZP_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(VEHICLE "PLANE" CACHE STRING "PLANE or QUADCOPTER")
# hwbuild.bash's -v flag (and its build/h753iit-<v>-<type> dir naming) only
# accepts "plane" or "quad", not a lowercased VEHICLE macro -- "quadcopter"
# would silently miss the library hwbuild.bash actually produced.
if(VEHICLE STREQUAL "QUADCOPTER")
    set(VEHICLE_LC "quad")
else()
    set(VEHICLE_LC "plane")
endif()
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LC)
set(ZP_LIB_DIR ${ZP_ROOT}/zeropilot4.0/build/h753iit-${VEHICLE_LC}-${BUILD_TYPE_LC})

file(GLOB_RECURSE HAL_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/*.c)
list(FILTER HAL_SRC EXCLUDE REGEX "_template\\.c$")

file(GLOB CORE_C   ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c)
file(GLOB CORE_CPP ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.cpp)
file(GLOB_RECURSE BOARD_SRC
     ${CMAKE_CURRENT_SOURCE_DIR}/boardfiles/*.c
     ${CMAKE_CURRENT_SOURCE_DIR}/boardfiles/*.cpp)

set(FR ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source)
file(GLOB FREERTOS_SRC ${FR}/*.c ${FR}/CMSIS_RTOS_V2/*.c)
list(APPEND FREERTOS_SRC ${FR}/portable/GCC/ARM_CM4F/port.c ${FR}/portable/MemMang/heap_4.c)

set(FATFS ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FatFs/src)
set(FATFS_SRC ${FATFS}/ff.c ${FATFS}/diskio.c ${FATFS}/ff_gen_drv.c ${FATFS}/option/syscall.c
              ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/App/fatfs.c
              ${CMAKE_CURRENT_SOURCE_DIR}/FATFS/Target/user_diskio.c)

file(GLOB DRONECAN_SRC ${ZP_ROOT}/external/dronecan/generated/src/*.c)
list(APPEND DRONECAN_SRC ${ZP_ROOT}/external/dronecan/libcanard/canard.c)

# CMSIS-DSP: CubeIDE's build pulls these in via its own include paths but never
# compiles the sources -- build the aggregate translation units for the
# function groups ZeroPilot actually references. Without these the link fails
# with "undefined reference to arm_mat_init_f32" / "dangerous relocation:
# unsupported relocation".
set(DSP ${ZP_ROOT}/external/CMSIS-DSP/Source)
set(DSP_SRC
    ${DSP}/BasicMathFunctions/BasicMathFunctions.c
    ${DSP}/CommonTables/CommonTables.c
    ${DSP}/ComplexMathFunctions/ComplexMathFunctions.c
    ${DSP}/FastMathFunctions/FastMathFunctions.c
    ${DSP}/FilteringFunctions/FilteringFunctions.c
    ${DSP}/MatrixFunctions/MatrixFunctions.c
    ${DSP}/QuaternionMathFunctions/QuaternionMathFunctions.c
    ${DSP}/StatisticsFunctions/StatisticsFunctions.c
    ${DSP}/SupportFunctions/SupportFunctions.c
    ${DSP}/TransformFunctions/TransformFunctions.c)

add_executable(${PROJECT_NAME}
    ${CORE_C} ${CORE_CPP} ${HAL_SRC} ${BOARD_SRC}
    ${FREERTOS_SRC} ${FATFS_SRC} ${DRONECAN_SRC} ${DSP_SRC}
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Startup/startup_stm32h753iitx.s)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER STM32H753xx USE_PWR_LDO_SUPPLY DEBUG ${VEHICLE})

target_include_directories(${PROJECT_NAME} PRIVATE
    Core/Inc
    Drivers/STM32H7xx_HAL_Driver/Inc Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32H7xx/Include Drivers/CMSIS/Include
    ${FR}/include ${FR}/CMSIS_RTOS_V2 ${FR}/portable/GCC/ARM_CM4F
    ${FATFS} FATFS/App FATFS/Target
    boardfiles/utils boardfiles/model/inc boardfiles/rtos/museq boardfiles/rtos/threads/inc
    boardfiles/drivers/barometer boardfiles/drivers/can_controller boardfiles/drivers/fft
    boardfiles/drivers/gps boardfiles/drivers/imu boardfiles/drivers/iwdg
    boardfiles/drivers/logger boardfiles/drivers/mathutils boardfiles/drivers/motor
    boardfiles/drivers/power_module boardfiles/drivers/queue boardfiles/drivers/rangefinder
    boardfiles/drivers/rc boardfiles/drivers/safety_switch boardfiles/drivers/systemutils
    boardfiles/drivers/telemlink
    ${ZP_ROOT}/zeropilot4.0 ${ZP_LIB_DIR} ${ZP_LIB_DIR}/include
    ${ZP_ROOT}/external/dronecan/generated/include ${ZP_ROOT}/external/dronecan/libcanard)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
    ${ZP_ROOT}/external/c_library_v2/all
    ${ZP_ROOT}/external/CMSIS-DSP/Include ${ZP_ROOT}/external/CMSIS-DSP/PrivateInclude)

target_link_directories(${PROJECT_NAME} PRIVATE ${ZP_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE zeropilot4.0 c m nosys)
target_link_options(${PROJECT_NAME} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32H753IITX_FLASH.ld
    -Wl,-Map=${PROJECT_NAME}.map,--cref,--gc-sections
    -static -Wl,--start-group -Wl,--end-group)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

cmake_minimum_required(VERSION 3.22)

#
# motor-ctrl 固件（STM32F030K6）— Hall6 闭环 6-step（默认）
#

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(TOOLCHAIN_BIN_DIR "")
file(GLOB _ARM_TOOLCHAIN_GCCS "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-gcc")
if(_ARM_TOOLCHAIN_GCCS)
  list(SORT _ARM_TOOLCHAIN_GCCS)
  list(REVERSE _ARM_TOOLCHAIN_GCCS)
  foreach(_gcc_candidate IN LISTS _ARM_TOOLCHAIN_GCCS)
    get_filename_component(_candidate_bin_dir "${_gcc_candidate}" DIRECTORY)
    get_filename_component(_candidate_root "${_candidate_bin_dir}" DIRECTORY)
    if(EXISTS "${_candidate_root}/arm-none-eabi/include/stdint.h")
      set(TOOLCHAIN_BIN_DIR "${_candidate_bin_dir}")
      break()
    endif()
  endforeach()
endif()

if(TOOLCHAIN_BIN_DIR)
  set(CMAKE_C_COMPILER   "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc" CACHE FILEPATH "" FORCE)
  set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc" CACHE FILEPATH "" FORCE)
  set(CMAKE_OBJCOPY      "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy" CACHE FILEPATH "" FORCE)
  set(CMAKE_SIZE         "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-size" CACHE FILEPATH "" FORCE)
else()
  set(CMAKE_C_COMPILER arm-none-eabi-gcc)
  set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
  set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
  set(CMAKE_SIZE arm-none-eabi-size)
endif()

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(motor-ctrl C ASM)

set(MCU_FLAGS -mcpu=cortex-m0 -mthumb)
option(MOTOR_AUTO_START "Enable motor on boot with default CCW params" OFF)
option(MOTOR_PHASE_SWEEP "Rotate hall phase offset every ~2s during openloop" OFF)
option(MOTOR_UVW_SWEEP "Rotate U/V/W pin remap every ~2s during openloop" OFF)
option(MOTOR_HS_ONLY "Diagnostic: drive PWM leg high side only (no complementary low side)" OFF)
option(MOTOR_START_HALL6 "Auto-start hall closed loop instead of open loop" OFF)
option(MOTOR_START_DIAG "Auto-start one static commutation step (no rotation)" OFF)
option(MOTOR_START_BENCH "Run the control-ISR cycle bench at boot (motor off)" OFF)
option(MOTOR_START_FOC_OBSERVE "Auto-start hall6 with FOC observing only (stage E step 1)" OFF)
option(MOTOR_ADC "PWM-synchronous phase current + bus voltage sampling (stage C)" ON)
option(MOTOR_ANGLE "Continuous rotor electrical angle from the halls (stage D)" ON)
option(MOTOR_FOC "Hall-sensored field-oriented control (stage E)" ON)
# nFAULT_MCU through soldered JP2, active-low into TIM1 BKIN.
# The 2026-09-12 trips (duty 0, shunts ~0) were taken as a false window
# comparator and BKIN was turned off. After the V-path UCC27211 died,
# that reading is probably wrong: PWM-synchronous shunts cannot see
# edge/shoot-through, the analog comparator can. Default ON. If it
# still trips on a healthy board, debug JP2/threshold — do not turn
# this off again without new evidence.
option(MOTOR_PWM_BKIN "Arm TIM1 BKIN from nFAULT_MCU (JP2)" ON)
set(MOTOR_FOC_DECIM "" CACHE STRING "Control ticks per FOC step (default 2 = 10 kHz)")
set(MOTOR_FOC_WC_HZ "" CACHE STRING "Current loop bandwidth in Hz (default 800)")
set(MOTOR_FOC_LAMBDA "" CACHE STRING "Back-EMF feed-forward flux linkage in Wb (default 0)")
set(MOTOR_ANGLE_OFFSET_Q16 "" CACHE STRING
    "Hall anchor to electrical angle offset, 65536 per revolution (stage E calibration)")
option(MOTOR_ADC_STROBE "Pulse TP1 (PA15) at end of ADC sequence for scope checks" OFF)
option(MOTOR_TRACE "On-chip per-tick sample buffer (stage A1); costs ~2 KB RAM" ON)
set(MOTOR_ADC_LEAD_COUNTS "" CACHE STRING
    "Timer counts before the centre-aligned peak at which the ADC sequence starts (default 72 = 1.5 us)")
set(MOTOR_TRACE_DEPTH "" CACHE STRING "Trace samples per channel (default 256)")
set(MOTOR_TRACE_DECIM "" CACHE STRING "Keep one sample every N control ticks (default 1)")
# The stage-B float FOC from sim/codegen_stm32 costs ~12 KB flash and ~0.5 KB RAM
# (soft-float libm). Its verdict is already recorded -- 15~18k cycles, see
# docs/mainboard-spec.md section 0 -- so keep it out of the runtime image and
# only link it when re-running the timing bench.
option(MOTOR_FOC_BENCH "Link sim/codegen_stm32 float FOC and its timing bench" OFF)
set(MOTOR_FOC_IQ_LSB "" CACHE STRING "FOC |iq| clamp in ADC LSB (default 186 = 300 mA)")
set(MOTOR_FOC_ID_LSB "" CACHE STRING "FOC |id| clamp in ADC LSB (default = MOTOR_FOC_IQ_LSB)")
set(MOTOR_HALL6_KICK_DUTY "" CACHE STRING "Hall6 kick duty percent (default 6)")
set(MOTOR_HALL6_RUN_DUTY "" CACHE STRING "Hall6 run duty percent (default 7)")
set(MOTOR_HALL6_PHASE "" CACHE STRING "Hall6 phase offset 0..5 (default 3)")
set(MOTOR_HALL6_CCW "" CACHE STRING "Hall6 direction, 1 = CCW (default 0)")
set(MOTOR_DUTY_PCT "" CACHE STRING "Openloop duty percent (default 7)")
set(MOTOR_TIM1_DTG "" CACHE STRING "TIM1 BDTR DTG raw value (default 72 = 1.5us)")
set(MOTOR_PULSE_COUNTS "" CACHE STRING "Openloop CCR in timer counts (0 = use duty percent)")
set(MOTOR_STEP_MS "" CACHE STRING "Openloop ms per commutation step (default 20)")
set(MOTOR_PHASE_OFFSET "" CACHE STRING "Openloop hall phase offset 0..5 (default 3)")

add_compile_options(${MCU_FLAGS}
  -Wall -Wextra -ffunction-sections -fdata-sections
  $<$<CONFIG:Debug>:-Og> $<$<CONFIG:Debug>:-g3>
  $<$<CONFIG:Release>:-Os>)

add_compile_definitions(
  STM32F030x6
  USE_HAL_DRIVER)

if(MOTOR_AUTO_START)
  add_compile_definitions(MOTOR_AUTO_START=1)
endif()
if(MOTOR_PHASE_SWEEP)
  add_compile_definitions(MOTOR_PHASE_SWEEP=1)
endif()
if(MOTOR_UVW_SWEEP)
  add_compile_definitions(MOTOR_UVW_SWEEP=1)
endif()
if(MOTOR_HS_ONLY)
  add_compile_definitions(MOTOR_HS_ONLY=1)
endif()
if(MOTOR_START_HALL6)
  add_compile_definitions(MOTOR_START_HALL6=1)
endif()
if(MOTOR_START_DIAG)
  add_compile_definitions(MOTOR_START_DIAG=1)
endif()
if(MOTOR_START_BENCH)
  add_compile_definitions(MOTOR_START_BENCH=1)
endif()
if(MOTOR_START_FOC_OBSERVE)
  add_compile_definitions(MOTOR_START_FOC_OBSERVE=1)
endif()
if(MOTOR_TRACE)
  add_compile_definitions(MOTOR_TRACE=1)
endif()
if(MOTOR_ADC)
  add_compile_definitions(MOTOR_ADC=1)
endif()
if(MOTOR_ADC_STROBE)
  add_compile_definitions(MOTOR_ADC_STROBE=1)
endif()
if(MOTOR_ANGLE)
  add_compile_definitions(MOTOR_ANGLE=1)
endif()
if(MOTOR_FOC)
  add_compile_definitions(MOTOR_FOC=1)
endif()
if(MOTOR_PWM_BKIN)
  add_compile_definitions(MOTOR_PWM_BKIN=1)
else()
  add_compile_definitions(MOTOR_PWM_BKIN=0)
endif()
foreach(_motor_param MOTOR_DUTY_PCT MOTOR_TIM1_DTG MOTOR_PULSE_COUNTS MOTOR_STEP_MS
                     MOTOR_PHASE_OFFSET MOTOR_HALL6_KICK_DUTY MOTOR_HALL6_RUN_DUTY
                     MOTOR_HALL6_PHASE MOTOR_HALL6_CCW
                     MOTOR_TRACE_DEPTH MOTOR_TRACE_DECIM MOTOR_ADC_LEAD_COUNTS
                     MOTOR_ANGLE_OFFSET_Q16 MOTOR_FOC_DECIM MOTOR_FOC_IQ_LSB
                     MOTOR_FOC_ID_LSB)
foreach(_motor_float MOTOR_FOC_WC_HZ MOTOR_FOC_LAMBDA)
  if(NOT "${${_motor_float}}" STREQUAL "")
    add_compile_definitions(${_motor_float}=${${_motor_float}}f)
  endif()
endforeach()
  if(NOT "${${_motor_param}}" STREQUAL "")
    add_compile_definitions(${_motor_param}=${${_motor_param}}U)
  endif()
endforeach()

set(PLAT_DIR ${CMAKE_SOURCE_DIR}/../../platforms/stm32f030)
set(FOC_DIR  ${CMAKE_SOURCE_DIR}/../../../sim/codegen_stm32)

file(GLOB APP_SRC  ${CMAKE_SOURCE_DIR}/App/*.c)
file(GLOB CORE_SRC ${CMAKE_SOURCE_DIR}/Core/Src/*.c)
file(GLOB PLAT_SRC ${PLAT_DIR}/*.c)

if(NOT MOTOR_TRACE)
  # Drop the translation unit rather than rely on --gc-sections: the buffer is a
  # 2 KB global and it should be provably absent, not probably collected.
  list(REMOVE_ITEM PLAT_SRC ${PLAT_DIR}/motor_trace.c)
endif()
if(NOT MOTOR_ADC)
  list(REMOVE_ITEM PLAT_SRC ${PLAT_DIR}/motor_adc.c)
endif()
if(NOT MOTOR_ANGLE)
  list(REMOVE_ITEM PLAT_SRC ${PLAT_DIR}/motor_angle.c)
endif()
if(NOT MOTOR_FOC)
  list(REMOVE_ITEM PLAT_SRC ${PLAT_DIR}/motor_foc.c)
  list(REMOVE_ITEM PLAT_SRC ${PLAT_DIR}/motor_foc_fx.c)
  set(FOC_FX_LINK_OPTS "")
else()
  # --gc-sections would drop MotorFocFx_Step from the default image, where
  # nothing calls it yet. The 5.1 objdump check needs the symbol present.
  set(FOC_FX_LINK_OPTS -Wl,--undefined=MotorFocFx_Step)
endif()

if(MOTOR_FOC_BENCH)
  add_compile_definitions(MOTOR_FOC_BENCH=1)
  set(FOC_SRC
    ${FOC_DIR}/foc_step_stm32.c
    ${FOC_DIR}/foc_step_stm32_data.c
    ${FOC_DIR}/foc_step_stm32_initialize.c)
  set(FOC_LINK_OPTS -Wl,--undefined=MotorFocBench_Run)
else()
  list(REMOVE_ITEM APP_SRC
    ${CMAKE_SOURCE_DIR}/App/foc_bench.c
    ${CMAKE_SOURCE_DIR}/App/foc_stubs.c)
  set(FOC_SRC "")
  set(FOC_LINK_OPTS "")
endif()

if(NOT MOTOR_START_BENCH)
  list(REMOVE_ITEM APP_SRC ${CMAKE_SOURCE_DIR}/App/ctrl_bench.c)
endif()

set(HAL_SRC
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_cortex.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_rcc_ex.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_flash_ex.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_gpio.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_dma.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_pwr_ex.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_exti.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_tim_ex.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc.c
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Src/stm32f0xx_hal_adc_ex.c
)

add_executable(${PROJECT_NAME}.elf
  ${APP_SRC}
  ${CORE_SRC}
  ${PLAT_SRC}
  ${FOC_SRC}
  ${HAL_SRC}
  ${CMAKE_SOURCE_DIR}/startup_stm32f030x6.s)

target_include_directories(${PROJECT_NAME}.elf PRIVATE
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/App
  ${CMAKE_SOURCE_DIR}/Core/Inc
  ${PLAT_DIR}
  ${FOC_DIR}
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Inc
  ${CMAKE_SOURCE_DIR}/Drivers/STM32F0xx_HAL_Driver/Inc/Legacy
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F0xx/Include
  ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include)

target_link_options(${PROJECT_NAME}.elf PRIVATE
  ${MCU_FLAGS}
  -T${CMAKE_SOURCE_DIR}/STM32F030X6.ld
  --specs=nano.specs
  --specs=nosys.specs
  -Wl,--gc-sections
  -Wl,--undefined=MotorApp_Start
  -Wl,--undefined=MotorApp_Stop
  -Wl,--undefined=MotorApp_StartOpenloop6
  -Wl,--undefined=MotorApp_DiagGPhase
  -Wl,--undefined=MotorApp_DiagStop
  -Wl,--undefined=MotorHall6_Enable
  ${FOC_FX_LINK_OPTS}
  ${FOC_LINK_OPTS}
  -lm
  -Wl,--print-memory-usage
  -Wl,-Map=${PROJECT_NAME}.map)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>)

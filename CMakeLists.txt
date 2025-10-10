cmake_minimum_required(VERSION 3.20)
project(betaflight LANGUAGES C CXX VERSION 1)

set(CMAKE_C_VISIBILITY_PRESET hidden)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fenable-matrix")
set(  CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -fenable-matrix")

set(CMAKE_SIZEOF_VOID_P 8)  # Enforces 64-bit pointer size

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")  # TEMPORARY WHILE DEBUGGING

add_library(betaflight_SITL SHARED
  src/main/blackbox/blackbox_encoding.c
  src/main/blackbox/blackbox_io.c
  src/main/blackbox/blackbox_virtual.c
  src/main/blackbox/blackbox.c
  src/main/build/build_config.c
  src/main/build/debug.c
  src/main/build/version.c
  src/main/cli/cli.c
  src/main/cli/settings.c
  src/main/cms/cms_menu_blackbox.c
  src/main/cms/cms_menu_failsafe.c
  src/main/cms/cms_menu_firmware.c
  src/main/cms/cms_menu_firmware.c
  src/main/cms/cms_menu_gps_rescue_multirotor.c
  src/main/cms/cms_menu_imu.c
  src/main/cms/cms_menu_main.c
  src/main/cms/cms_menu_misc.c
  src/main/cms/cms_menu_osd.c
  src/main/cms/cms_menu_persistent_stats.c
  src/main/cms/cms_menu_power.c
  src/main/cms/cms_menu_saveexit.c
  src/main/cms/cms_menu_vtx_common.c
  src/main/cms/cms_menu_vtx_msp.c
  src/main/cms/cms.c
  src/main/common/bitarray.c
  src/main/common/crc.c
  src/main/common/encoding.c
  src/main/common/filter.c
  src/main/common/gps_conversion.c
  src/main/common/maths.c
  src/main/common/printf_serial.c
  src/main/common/printf.c
  src/main/common/sensor_alignment.c
  src/main/common/streambuf.c
  src/main/common/time.c
  src/main/common/typeconversion.c
  src/main/common/utils.c
  src/main/common/vector.c
  src/main/config/config_eeprom.c
  src/main/config/config_streamer.c
  src/main/config/config.c
  src/main/config/feature.c
  src/main/config/simplified_tuning.c
  src/main/drivers/accgyro/accgyro_virtual.c
  src/main/drivers/accgyro/gyro_sync.c
  src/main/drivers/adc.c
  src/main/drivers/barometer/barometer_virtual.c
  src/main/drivers/buf_writer.c
  src/main/drivers/bus.c
  src/main/drivers/display.c
  src/main/drivers/dshot.c
  src/main/drivers/io.c
  src/main/drivers/motor.c
  src/main/drivers/pwm_output.c
  src/main/drivers/serial_direct.c
  src/main/drivers/serial.c
  src/main/drivers/stack_check.c
  src/main/drivers/timer_common.c
  src/main/drivers/vtx_common.c
  src/main/drivers/vtx_table.c
  src/main/fc/board_info.c
  src/main/fc/controlrate_profile.c
  src/main/fc/core.c
  src/main/fc/dispatch.c
  src/main/fc/init.c
  src/main/fc/rc_adjustments.c
  src/main/fc/rc_controls.c
  src/main/fc/rc_modes.c
  src/main/fc/rc.c
  src/main/fc/runtime_config.c
  src/main/fc/stats.c
  src/main/fc/tasks.c
  src/main/flight/autopilot_multirotor.c
  src/main/flight/failsafe.c
  src/main/flight/gps_rescue_multirotor.c
  src/main/flight/imu.c
  src/main/flight/mixer_init.c
  src/main/flight/mixer.c
  src/main/flight/pid_init.c
  src/main/flight/pid.c
  src/main/flight/position.c
  src/main/io/beeper.c
  src/main/io/displayport_msp.c
  src/main/io/gps_virtual.c
  src/main/io/gps.c
  src/main/io/rcdevice_cam.c
  src/main/io/rcdevice.c
  src/main/io/serial_resource.c
  src/main/io/serial.c
  src/main/io/statusindicator.c
  src/main/io/vtx_control.c
  src/main/io/vtx_msp.c
  src/main/io/vtx.c
  src/main/main.c
  src/main/msp/msp_box.c
  src/main/msp/msp_build_info.c
  src/main/msp/msp_serial.c
  src/main/msp/msp.c
  src/main/osd/osd_elements.c
  src/main/osd/osd_warnings.c
  src/main/osd/osd.c
  src/main/pg/adc.c
  src/main/pg/autopilot_multirotor.c
  src/main/pg/board.c
  src/main/pg/displayport_profiles.c
  src/main/pg/gps_lap_timer.c
  src/main/pg/gps_rescue_multirotor.c
  src/main/pg/gps.c
  src/main/pg/gyrodev.c
  src/main/pg/motor.c
  src/main/pg/msp.c
  src/main/pg/pg.c
  src/main/pg/pilot.c
  src/main/pg/rcdevice.c
  src/main/pg/rx.c
  src/main/pg/scheduler.c
  src/main/pg/stats.c
  src/main/pg/vcd.c
  src/main/pg/vtx_table.c
  src/main/rx/msp_override.c
  src/main/rx/msp.c
  src/main/rx/rx.c
  src/main/scheduler/scheduler.c
  src/main/sensors/acceleration_init.c
  src/main/sensors/acceleration.c
  src/main/sensors/adcinternal.c
  src/main/sensors/barometer.c
  src/main/sensors/battery.c
  src/main/sensors/boardalignment.c
  src/main/sensors/compass.c
  src/main/sensors/current.c
  src/main/sensors/gyro_init.c
  src/main/sensors/gyro.c
  src/main/sensors/initialisation.c
  src/main/sensors/voltage.c
  src/platform/SIMULATOR/sitl.c
  physics/physics_copy.c
)

target_include_directories(betaflight_SITL PRIVATE
  src/platform/SIMULATOR
  src/platform/SIMULATOR/include
  src/platform/SIMULATOR/target/SITL
  src/main
)

target_compile_definitions(betaflight_SITL PUBLIC
  SIMULATOR_BUILD=1
  SITL
  USE_GPS=1
  USE_OSD=1
  USE_OSD_HD=1
  USE_MSP_DISPLAYPORT=
  USE_VTX=1
  USE_VTX_MSP=
  USE_VTX_COMMON=
  USE_VTX_CONTROL=
  USE_BLACKBOX=
  USE_ADC=
  USE_DETERMINISM=
  USE_CMS= # Needed for OSD, some lines check for this and OSD over MSP
  _GNU_SOURCE
  __TARGET__="SITL"
  __REVISION__="0"
  __FORKNAME__="SITL_WASM"
  _WASI_EMULATED_SIGNAL
  _WASI_EMULATED_PTHREAD
  HSE_VALUE=8000000
  FLASH_SIZE=2048
)

target_compile_options(betaflight_SITL PRIVATE
  -Wno-parentheses-equality
  -Wno-unknown-pragmas
  -Wno-incompatible-pointer-types
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Configuring for a DEBUG build")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  set(  CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} -fno-optimize-sibling-calls -fno-omit-frame-pointer")
  target_compile_definitions(betaflight_SITL PUBLIC
    USE_FLUSH_IO=
  )
else()
  message(STATUS "Configuring for a RELEASE build")
endif()

set_property(TARGET betaflight_SITL PROPERTY POSITION_INDEPENDENT_CODE ON)
target_compile_features(betaflight_SITL PUBLIC cxx_std_17)

if(CMAKE_SYSTEM_NAME STREQUAL "WASI")
  string(REPLACE "-Wl,--import-memory" "" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--error-limit=0 -ferror-limit=0 -Wl,--initial-memory=10485760")

  target_link_libraries(betaflight_SITL PRIVATE
    wasi-emulated-signal
    wasi-emulated-pthread
  )
  target_compile_definitions(betaflight_SITL PUBLIC
    WASI
  )
else()
  target_link_options(betaflight_SITL PRIVATE
    "-Wl,-T${CMAKE_SOURCE_DIR}/src/platform/SIMULATOR/link/sitl.ld"
  )
  target_link_libraries(betaflight_SITL PRIVATE
    m
  )
endif()

#target_link_options(betaflight_SITL PRIVATE
#  -mexec-model=reactor
#)

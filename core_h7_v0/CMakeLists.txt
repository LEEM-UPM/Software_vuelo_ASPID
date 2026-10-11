cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME core_h7_v0)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# 2. Global Configuration Definition
add_library(project_config INTERFACE)
target_include_directories(project_config INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/config)
target_compile_definitions(project_config INTERFACE 
    TX_INCLUDE_USER_DEFINE_FILE
    FX_INCLUDE_USER_DEFINE_FILE
)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)
add_subdirectory(third_party/middlewares/threadx)
add_subdirectory(third_party/middlewares/filex)
# bsp/filex (driver SD con ThreadX) solo lo usa la PoC poc/usd_filex_dma;
# el ordenador de vuelo usa Drivers/Sd_card con FileX standalone.
#add_subdirectory(bsp)
add_subdirectory(third_party/can_protocol)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    #poc/can_tx_rx_rtos/main.c
    #poc/can_tx_rx_rtos/can_rx_thread.c
    #poc/can_tx_rx_rtos/can_shared_data.c
    #poc/can_tx_rx_rtos/can_tx_test.c
    # App: lógica del ordenador de vuelo
    App/flight_software.c
    App/sensor_board.c
    App/sys_utilities.c
    App/sys_power.c
    App/sys_avionics.c
    App/sys_rf.c
    App/debug_console.c
    App/storage.c
    App/sounds.c
    App/led_show.c
    App/gnc.c
    App/rf_telemetry.c
    App/flight_packet.c
    App/status_packet.c
    App/uplink.c
    App/despliegue.c
    App/lanzamiento.c
    App/storage_stream.c
    App/pil_test.c
    App/airbrake.c
    App/gnc_playback.c
    App/sd_bench.c
    App/aspid_data.c
    App/camara.c
    App/memoria_vuelo.c
    App/baro_alt.c
    App/emergencia.c
    App/watchdog.c
    App/boton_memoria.c
    App/camaras.c
    # Drivers propios
    Drivers/Buzzer/buzzer.c
    Drivers/Leds/leds.c
    Drivers/Runcam/runcam.c
    Drivers/Can_driver/can_driver.c
    Drivers/Debug_out/debug_out.c
    Drivers/Sd_card/sd_card.c
    Drivers/Sd_card/sd_card_fx_driver.c
    Drivers/Usb_cdc/usb_cdc.c
    Drivers/Usb_cdc/usbd_conf.c
    Drivers/Usb_cdc/usbd_desc.c
    # Librería USB Device de ST (núcleo + clase CDC)
    third_party/stm32_usb_device_library/Core/Src/usbd_core.c
    third_party/stm32_usb_device_library/Core/Src/usbd_ctlreq.c
    third_party/stm32_usb_device_library/Core/Src/usbd_ioreq.c
    third_party/stm32_usb_device_library/Class/CDC/Src/usbd_cdc.c
    # GNC generado con Simulink (ASPID v1.6): navegacion, maquina de estados y
    # guiado. shared/ va una sola vez (ver third_party/aspid_gnc/README.md)
    third_party/aspid_gnc/nav/nav.c
    third_party/aspid_gnc/nav/nav_data.c
    third_party/aspid_gnc/state_machine/state_machine.c
    third_party/aspid_gnc/gc/gc.c
    third_party/aspid_gnc/gc/gc_data.c
    third_party/aspid_gnc/shared/rt_nonfinite.c
    third_party/aspid_gnc/shared/rtGetInf.c
    third_party/aspid_gnc/shared/rtGetNaN.c
)

# El codigo generado (desde la v1.2, y sigue en la v1.6) define rt_powd_snf()
# sin static en nav.c Y en gc.c (identicas), y el enlazado da "multiple
# definition". Sin tocar el codigo generado: la de gc.c se compila con otro
# nombre. Hay que avisar a GNC (PENDIENTES); si la proxima entrega la saca a
# shared/, quitar esto.
set_source_files_properties(third_party/aspid_gnc/gc/gc.c PROPERTIES
    COMPILE_DEFINITIONS "rt_powd_snf=gc_rt_powd_snf")

# El GNC corre a 416 Hz (2,4 ms por tick) y el EKF de la v1.2 a -O0 tardaba
# ~10 ms:
# el codigo generado se optimiza siempre, tambien en Debug. Y el registro de
# la SD (aspid_data.c), que formatea ~90 numeros por fila a 100 Hz en el
# mismo bucle. El resto del
# proyecto sigue a -O0 para poder depurarlo. Sin -ffast-math: el modelo mira
# NaN e infinitos (rtIsNaN) y el banco PIL compara con tolerancias finas.
#
# -ffp-contract=off (2026-10-11, GNC v1.6): a -O2 el GCC de ARM junta
# multiplicacion y suma en una sola instruccion (FMA), que redondea distinto.
# Con eso el banco PIL en la placa daba 751 discrepancias en el guiado (solo
# en reales: apogeo previsto, error, mando...; ninguna en fases ni banderas);
# sin FMA, 0, igual que el modelo y que el PC. Cuesta ~0,1 ms en el peor tick.
set_source_files_properties(
    third_party/aspid_gnc/nav/nav.c
    third_party/aspid_gnc/nav/nav_data.c
    third_party/aspid_gnc/state_machine/state_machine.c
    third_party/aspid_gnc/gc/gc.c
    third_party/aspid_gnc/gc/gc_data.c
    third_party/aspid_gnc/shared/rt_nonfinite.c
    third_party/aspid_gnc/shared/rtGetInf.c
    third_party/aspid_gnc/shared/rtGetNaN.c
    App/gnc.c
    App/aspid_data.c
    PROPERTIES COMPILE_OPTIONS "-O2;-ffp-contract=off")

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    Drivers/Buzzer
    Drivers/Can_driver
    Drivers/Debug_out
    Drivers/Leds
    Drivers/Runcam
    Drivers/Sd_card
    Drivers/Usb_cdc
    third_party/stm32_usb_device_library/Core/Inc
    third_party/stm32_usb_device_library/Class/CDC/Inc
    third_party/aspid_gnc/nav
    third_party/aspid_gnc/state_machine
    third_party/aspid_gnc/gc
    third_party/aspid_gnc/shared
)

# newlib-nano no formatea %f sin esto (consola de debug)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

target_link_libraries(${PROJECT_NAME}
    threadx
    filex
    stm32cubemx
    can_protocol
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
# cmake_minimum_required(VERSION 3.24)

# project(stm32l5_drivers LANGUAGES C)

# set(CMAKE_C_STANDARD 20)


set(STM32L5_CMSIS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS)
set(STM32L5_CMSIS_INC_DIR ${STM32L5_CMSIS_DIR}/Include)
set(STM32L5_CMSIS_DEVICE_INC_DIR ${STM32L5_CMSIS_DIR}/Device/ST/STM32L5xx/Include)
set(STM32L5_CMSIS_DSP_DIR ${STM32L5_CMSIS_DIR}/DSP)
set(STM32L5_CMSIS_DSP_INC_DIR ${STM32L5_CMSIS_DSP_DIR}/Include)


add_library(
    stm32l5xx_cmsis
    INTERFACE
)

target_include_directories(
    stm32l5xx_cmsis
    INTERFACE
    ${STM32L5_CMSIS_INC_DIR}
    ${STM32L5_CMSIS_DEVICE_INC_DIR}
    ${STM32L5_CMSIS_DSP_INC_DIR}
)

# set_target_properties(stm32l5xx_cmsis PROPERTIES IMPORTED_LOCATION ${STM32L5_CMSIS_DSP_DIR}/Lib/GCC/libarm_ARMv8MMLl_math.a)
add_library(stm32l5xx_cmdis_dsp STATIC IMPORTED GLOBAL)
set_target_properties(stm32l5xx_cmdis_dsp PROPERTIES IMPORTED_LOCATION ${STM32L5_CMSIS_DSP_DIR}/Lib/GCC/libarm_ARMv8MMLldfsp_math.a)
# target_include_directories(stm32l5xx_cmdis_dsp INTERFACE ${STM32L5_CMSIS_DSP_INC_DIR})




set(STM32L5_HAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/stm32l5xx_hal_driver)
set(STM32L5_HAL_SRC_DIR ${STM32L5_HAL_DIR}/Src)
set(STM32L5_HAL_INC_DIR ${STM32L5_HAL_DIR}/Inc)



add_library(
    stm32l5xx_hal
    STATIC EXCLUDE_FROM_ALL
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_adc_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_adc.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_cortex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_crc_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_crc.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_cryp_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_cryp.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_dac_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_dac.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_dma_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_dma.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_exti.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_flash_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_flash_ramfunc.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_flash.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_gpio.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_i2c_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_i2c.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_icache.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_ospi.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_pwr_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_pwr.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_rcc_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_rcc.c
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_rng_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_rng.c
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_rtc_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_rtc.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_sd_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_sd.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_spi_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_spi.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_tim_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_tim.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_uart_ex.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal_uart.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_hal.c 
    ${STM32L5_HAL_SRC_DIR}/stm32l5xx_ll_sdmmc.c

)



target_include_directories(stm32l5xx_hal INTERFACE ${STM32L5_HAL_INC_DIR} PRIVATE ${STM32L5_HAL_INC_DIR})
target_include_directories(stm32l5xx_hal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/configs PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/configs)

# target_compile_options(stm32l5xx_hal PRIVATE -Wmissing-prototypes)
target_link_libraries(stm32l5xx_hal PRIVATE stm32l5xx_cmsis)


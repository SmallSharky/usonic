





add_executable(
    usonic

    main.c
    stm32l5xx_it.c
    stm32l5xx_hal_msp.c
    startup_stm32l562vetx.S
    syscalls.c
    sysmem.c
    system_stm32l5xx.c

    periph/adc.c
    periph/aes.c
    periph/crc.c
    periph/dac.c
    periph/dma.c
    periph/gpio.c
    periph/i2c.c
    periph/icache.c
    periph/octospi.c
    periph/rng.c
    periph/rtc.c
    periph/sdmmc.c
    periph/spi.c
    periph/tim.c
    periph/usart.c

    fatfs/src/diskio.c
    fatfs/src/ff_gen_drv.c
    fatfs/src/ff.c
    fatfs/src/option/ccsbcs.c
    fatfs/src/option/syscall.c

    sensors/BME280.c

    # wifi/wfx-fullmac-driver/bus/sl_wfx_bus_sdio.c
    wifi/wfx-fullmac-driver/bus/sl_wfx_bus_spi.c
    wifi/wfx-fullmac-driver/bus/sl_wfx_bus.c
    wifi/wfx-fullmac-driver/secure_link/sl_wfx_secure_link.c
    wifi/wfx-fullmac-driver/sl_wfx.c

    wifi/LwIP/src/api/api_lib.c
    wifi/LwIP/src/api/api_msg.c
    wifi/LwIP/src/api/err.c
    wifi/LwIP/src/api/if_api.c
    wifi/LwIP/src/api/netbuf.c
    wifi/LwIP/src/api/netdb.c
    wifi/LwIP/src/api/netifapi.c
    wifi/LwIP/src/api/sockets.c
    wifi/LwIP/src/api/tcpip.c
    # wifi/LwIP/src/apps/altcp_tls/altcp_tls_mbedtls_mem.c
    # wifi/LwIP/src/apps/altcp_tls/altcp_tls_mbedtls.c
    # wifi/LwIP/src/apps/lwiperf/lwiperf.c 
    # wifi/LwIP/src/apps/http/httpd.c 
    wifi/LwIP/src/apps/http/http_client.c 
    # wifi/LwIP/src/apps/http/fsdata.c 
    # wifi/LwIP/src/apps/http/fs.c 
    # wifi/LwIP/src/apps/http/altcp_proxyconnect.c 
    # wifi/LwIP/src/apps/http/makefsdata/makefsdata.c
    wifi/LwIP/src/core/altcp_alloc.c 
    wifi/LwIP/src/core/altcp_tcp.c
    wifi/LwIP/src/core/altcp.c 
    wifi/LwIP/src/core/def.c 
    wifi/LwIP/src/core/dns.c 
    wifi/LwIP/src/core/dns.h 
    wifi/LwIP/src/core/inet_chksum.c 
    wifi/LwIP/src/core/init.c 
    wifi/LwIP/src/core/ip.c 
    wifi/LwIP/src/core/mem.c 
    wifi/LwIP/src/core/memp.c 
    wifi/LwIP/src/core/netif.c 
    wifi/LwIP/src/core/pbuf.c 
    wifi/LwIP/src/core/raw.c 
    wifi/LwIP/src/core/stats.c 
    wifi/LwIP/src/core/sys.c 
    wifi/LwIP/src/core/tcp_in.c 
    wifi/LwIP/src/core/tcp_out.c 
    wifi/LwIP/src/core/tcp.c 
    wifi/LwIP/src/core/timeouts.c 
    wifi/LwIP/src/core/udp.c
    wifi/LwIP/src/core/ipv4/autoip.c 
    wifi/LwIP/src/core/ipv4/dhcp.c 
    wifi/LwIP/src/core/ipv4/etharp.c 
    wifi/LwIP/src/core/ipv4/icmp.c 
    wifi/LwIP/src/core/ipv4/igmp.c 
    wifi/LwIP/src/core/ipv4/ip4_addr.c 
    wifi/LwIP/src/core/ipv4/ip4_frag.c 
    wifi/LwIP/src/core/ipv4/ip4.c
    wifi/LwIP/src/core/ipv6/dhcp6.c
    wifi/LwIP/src/core/ipv6/ethip6.c 
    wifi/LwIP/src/core/ipv6/icmp6.c 
    wifi/LwIP/src/core/ipv6/inet6.c 
    wifi/LwIP/src/core/ipv6/ip6_addr.c 
    wifi/LwIP/src/core/ipv6/ip6_frag.c 
    wifi/LwIP/src/core/ipv6/ip6.c 
    wifi/LwIP/src/core/ipv6/mld6.c 
    wifi/LwIP/src/core/ipv6/nd6.c
    wifi/LwIP/src/netif/lowpan6_ble.c 
    wifi/LwIP/src/netif/lowpan6_common.c 
    wifi/LwIP/src/netif/lowpan6.c 
    wifi/LwIP/src/netif/slipif.c 
    wifi/LwIP/src/netif/zepif.c 
    wifi/LwIP/src/netif/ethernet.c 
    wifi/LwIP/src/netif/bridgeif.c 
    wifi/LwIP/src/netif/bridgeif_fdb.c

    wifi/dhcp_server.c 
    wifi/ethernetif.c 
    wifi/lwip_app.c 
    wifi/sl_wfx_host_spi.c 
    wifi/sl_wfx_host.c
    

    display/gifenc.c
    display/jpeg.c
    # display/JPEGENC.cpp
    display/lcd_mem.c
    display/utils.c
    


    fatfs/app_fatfs.c
    fatfs/fatfs_platform.c
    fatfs/sd_diskio.c
)


target_link_libraries(
    usonic
    PRIVATE

    stm32l5xx_cmsis
    stm32l5xx_cmdis_dsp
    stm32l5xx_hal
)


target_include_directories(
    usonic PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR} 
    ${CMAKE_CURRENT_SOURCE_DIR}/fatfs
    ${CMAKE_CURRENT_SOURCE_DIR}/fatfs/src
    ${CMAKE_CURRENT_SOURCE_DIR}/periph
    ${CMAKE_CURRENT_SOURCE_DIR}/display
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/wfx-fullmac-driver
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/wfx-fullmac-driver/bus
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/wfx-fullmac-driver/firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/wfx-fullmac-driver/secure_link
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/LwIP/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/LwIP/system
    ${CMAKE_CURRENT_SOURCE_DIR}/wifi/PDS
)

target_link_options(usonic PRIVATE -T ${CMAKE_CURRENT_SOURCE_DIR}/STM32L562VETX_FLASH.ld)


add_custom_command(
    TARGET usonic
    POST_BUILD
    BYPRODUCTS usonic.bin
    DEPENDS usonic
    COMMENT "generate .bin file"
    COMMAND ${OBJCOPY_BIN} -O binary "$<TARGET_FILE:usonic>" "$<TARGET_FILE:usonic>.bin"
    VERBATIM
)

add_custom_target(
    flash-usonic
    DEPENDS usonic.bin
    COMMAND st-flash write usonic.bin 0x08000000
)
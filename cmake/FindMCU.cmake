
add_definitions(

    -DSTM32L562xx
    -DARM_MATH_CM33
    -Dflash_layout
    -DSTM32L562VE
)

add_compile_options(
    -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16  -mfloat-abi=hard
    -fno-common
)

add_link_options(
    -mcpu=cortex-m33 -mthumb -mfpu=fpv5-sp-d16  -mfloat-abi=hard
    -fno-common
    --specs=nano.specs -u _printf_float --specs=nosys.specs
    -Wl,--no-warn-rwx-segments
)
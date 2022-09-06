
add_executable(pico_fram_test
  pico_fram_test.c
)

add_executable(pico_motor_test
  pico_motor_test.c
)

add_executable(pico_motor_analyze
  pico_motor_analyze.c
)

add_executable(pico_current_control
  pico_current_control.c
)

add_executable(pico_encoder_test
  pico_encoder_test.c
)

# add_executable(pico_imu_test
#   pico_imu_test.c
# )

# add_executable(pico_comms_test
#   pico_comms_test.c
# )

#add_executable(pico_serial_test
#  test_serial.c
#)

# pull in common dependencies
target_link_libraries(pico_fram_test
  pico_stdlib
  hardware_i2c
  rclib
)

target_link_libraries(pico_motor_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)

target_link_libraries(pico_motor_analyze
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)

target_link_libraries(pico_current_control
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
)

target_link_libraries(pico_encoder_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
)

# target_link_libraries(pico_imu_test
#   pico_stdlib
#   hardware_i2c
#   rclib
# )

# target_link_libraries(pico_comms_test
#   pico_stdlib
#   hardware_pwm
#   comms
# )

#target_link_libraries(pico_serial_test
#  pico_stdlib
#  hardware_pwm
#  pico_stdio_usb
#  comms
#)

# create map/bin/hex file etc.
pico_add_extra_outputs(pico_fram_test)
pico_add_extra_outputs(pico_motor_test)
pico_add_extra_outputs(pico_motor_analyze)
pico_add_extra_outputs(pico_current_control)
pico_add_extra_outputs(pico_encoder_test)
# pico_add_extra_outputs(pico_imu_test)
# pico_add_extra_outputs(pico_comms_test)
#pico_add_extra_outputs(pico_serial_test)

pico_enable_stdio_usb(pico_fram_test 1)
pico_enable_stdio_uart(pico_fram_test 0)
pico_enable_stdio_usb(pico_motor_test 1)
pico_enable_stdio_uart(pico_motor_test 0)
pico_enable_stdio_usb(pico_motor_analyze 1)
pico_enable_stdio_uart(pico_motor_analyze 0)
pico_enable_stdio_usb(pico_current_control 1)
pico_enable_stdio_uart(pico_current_control 0)
pico_enable_stdio_usb(pico_encoder_test 1)
pico_enable_stdio_uart(pico_encoder_test 0)
# pico_enable_stdio_usb(pico_imu_test 1)
# pico_enable_stdio_uart(pico_imu_test 0)
# pico_enable_stdio_usb(pico_comms_test 1)
# pico_enable_stdio_uart(pico_comms_test 0)
#pico_enable_stdio_usb(pico_serial_test 1)
#pico_enable_stdio_uart(pico_serial_test 0)

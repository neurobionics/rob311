include_directories(/usr/local/include)

#MATRIX-TESTS
add_executable(rc_matrix_test
rc_matrix_test.c
)
target_link_libraries(rc_matrix_test
  pico_stdlib
  rclib
)
pico_enable_stdio_usb(rc_matrix_test 1)
pico_add_extra_outputs(rc_matrix_test)


#FRAM-TEST
add_executable(mbot_fram_test
  mbot_fram_test.c
)
target_link_libraries(mbot_fram_test
  pico_stdlib
  hardware_i2c
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_fram_test 1)
pico_enable_stdio_uart(mbot_fram_test 0)
pico_add_extra_outputs(mbot_fram_test)

#BHY-TEST
add_executable(mbot_bhy_test
  mbot_bhy_test.c
)
target_link_libraries(mbot_bhy_test
  pico_stdlib
  hardware_i2c
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_bhy_test 1)
pico_enable_stdio_uart(mbot_bhy_test 0)
pico_add_extra_outputs(mbot_bhy_test)

#BARO-TEST
add_executable(mbot_barometer_test
  mbot_barometer_test.c
)
target_link_libraries(mbot_barometer_test
  mbotlib
  pico_stdlib
  hardware_i2c
  rclib
)
pico_enable_stdio_usb(mbot_barometer_test 1)
pico_enable_stdio_uart(mbot_barometer_test 0)
pico_add_extra_outputs(mbot_barometer_test)

#MOTOR-TEST
add_executable(mbot_motor_test
  mbot_motor_test.c
)
target_link_libraries(mbot_motor_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_motor_test 1)
pico_enable_stdio_uart(mbot_motor_test 0)
pico_add_extra_outputs(mbot_motor_test)

#READ-MOTOR-ADC-TEST
add_executable(mbot_read_motor_adc_test
  mbot_read_motor_adc_test.c
)
target_link_libraries(mbot_read_motor_adc_test
  pico_stdlib
  hardware_pwm
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_read_motor_adc_test 1)
pico_enable_stdio_uart(mbot_read_motor_adc_test 0)
pico_add_extra_outputs(mbot_read_motor_adc_test)

#MOTOR-ANALYZE
add_executable(mbot_motor_analyze
  mbot_motor_analyze.c
)
target_link_libraries(mbot_motor_analyze
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_motor_analyze 1)
pico_enable_stdio_uart(mbot_motor_analyze 0)
pico_add_extra_outputs(mbot_motor_analyze)

#I control tests
add_executable(mbot_current_control_test
  mbot_current_control_test.c
)
target_link_libraries(mbot_current_control_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  hardware_adc
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_current_control_test 1)
pico_enable_stdio_uart(mbot_current_control_test 0)
pico_add_extra_outputs(mbot_current_control_test)


#USB serial test
add_executable(mbot_usb_serial_test
  mbot_usb_serial_test.c
)
target_link_libraries(mbot_usb_serial_test
  pico_stdlib
  
)
pico_enable_stdio_usb(mbot_usb_serial_test 1)
pico_enable_stdio_uart(mbot_usb_serial_test 0)
pico_add_extra_outputs(mbot_usb_serial_test)

#Encoder tests
add_executable(mbot_encoder_test
  mbot_encoder_test.c
)
target_link_libraries(mbot_encoder_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
  mbotlib
)
pico_enable_stdio_usb(mbot_encoder_test 1)
pico_enable_stdio_uart(mbot_encoder_test 0)
pico_add_extra_outputs(mbot_encoder_test)

#interrupt test
add_executable(pico_interrupt_test
pico_interrupt_test.c
)
target_link_libraries(pico_interrupt_test
  pico_stdlib
  pico_multicore
  hardware_pio
  hardware_pwm
  rclib
  mbotlib
)
pico_enable_stdio_usb(pico_interrupt_test 1)
pico_enable_stdio_uart(pico_interrupt_test 0)
pico_add_extra_outputs(pico_interrupt_test)

#IMU test
add_executable(mbot_imu_test
  mbot_imu_test.c
)
target_link_libraries(mbot_imu_test
  pico_stdlib
  hardware_i2c
  mbotlib
  rclib
)
pico_enable_stdio_usb(mbot_imu_test 1)
pico_enable_stdio_uart(mbot_imu_test 0)
pico_add_extra_outputs(mbot_imu_test)

#Motor Calibration
add_executable(mbot_calibrate_classic
  mbot_calibrate_classic.c
)
target_link_libraries(mbot_calibrate_classic
  pico_stdlib
  hardware_i2c
  hardware_pio
  hardware_pwm
  mbotlib
  rclib
)
pico_enable_stdio_usb(mbot_calibrate_classic 1)
pico_enable_stdio_uart(mbot_calibrate_classic 0)
pico_add_extra_outputs(mbot_calibrate_classic)

#Motor Calibration
add_executable(mbot_calibrate_omni
  mbot_calibrate_omni.c
)
target_link_libraries(mbot_calibrate_omni
  pico_stdlib
  hardware_i2c
  hardware_pio
  hardware_pwm
  mbotlib
  rclib
)
pico_enable_stdio_usb(mbot_calibrate_omni 1)
pico_enable_stdio_uart(mbot_calibrate_omni 0)
pico_add_extra_outputs(mbot_calibrate_omni)

#-------------------------------------------------#

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
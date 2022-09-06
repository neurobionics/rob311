#MBOT.C
add_executable(test_dmp
  test_dmp.c
)

#MBOT.C
add_executable(bballbot
  bballbot.c
)

#MATRIX-TESTS.C
add_executable(matrix-tests
matrix-tests.c
)

# pull in common dependencies
target_link_libraries(test_dmp
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  comms
)

# pull in common dependencies
target_link_libraries(bballbot
  pico_stdlib
  pico_multicore
  hardware_pwm
  hardware_sync
  hardware_i2c
  hardware_flash
  rclib
  controls
  comms
)

# pull in common dependencies
target_link_libraries(matrix-tests
  pico_stdlib
  rclib
)

#Enable USB output for STDIO
pico_enable_stdio_usb(test_dmp 1)
pico_enable_stdio_uart(test_dmp 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(test_dmp)

#Enable USB output for STDIO
pico_enable_stdio_usb(bballbot 1)
pico_enable_stdio_uart(bballbot 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bballbot)

#Enable USB output for STDIO
pico_enable_stdio_usb(matrix-tests 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(matrix-tests)


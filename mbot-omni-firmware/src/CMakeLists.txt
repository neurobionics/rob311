#BBALLBOT.C
add_executable(bballbot
  bballbot.c
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
  mbotlib
)

#Enable USB output for STDIO
pico_enable_stdio_usb(bballbot 1)
pico_enable_stdio_uart(bballbot 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(bballbot)
add_library(controls STATIC
  src/controller.c
)

target_link_libraries(controls
  pico_stdlib
  pico_multicore
  hardware_pio
  rclib
  comms
)

target_include_directories(controls PUBLIC
  include
)
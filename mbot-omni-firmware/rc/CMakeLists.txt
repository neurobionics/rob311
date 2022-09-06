add_library(rclib STATIC
  # MATH
  src/math/algebra_common.c
  src/math/algebra.c
  src/math/filter.c
  src/math/kalman.c
  src/math/matrix.c
  src/math/other.c
  src/math/polynomial.c
  src/math/quaternion.c
  src/math/ring_buffer.c
  src/math/vector.c
  
  #src/math/test.c
  # MOTOR
  src/motor/motor.c
  # ENCODER
  src/encoder/encoder.c
  # IMU
  src/mpu/mpu.c
  # CURRENT SENSE ADC
  src/current/current.c
  # FRAM chip
  src/fram/fram.c
)

target_link_libraries(rclib
pico_stdlib
pico_multicore
hardware_pwm
hardware_i2c
hardware_pio
hardware_adc
hardware_flash
)

pico_generate_pio_header(rclib ${CMAKE_SOURCE_DIR}/rc/include/rc/encoder/quadrature_encoder.pio)

target_include_directories(rclib PUBLIC
  include
)

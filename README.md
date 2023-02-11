# Flight controller

![rendering](pictures/pcb.webp)

# Possible improvements

- Let the microcontroller control the reset pins of other chips. It is
  particularly annying when debugging that only the microcontroller gets reset
  using the programmer.
- Connect the SX1280 busy pin to the microcontroller.
- Add probe points:
  - GND
  - chip select
  - SDI
  - SDO
  - SCK
  - ...?

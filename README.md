# Flight controller

![rendering](pictures/pcb.webp)

# Possible improvements

- Let the microcontroller control the reset pins of other chips. It is
  particularly annying when debugging that only the microcontroller gets reset
  using the programmer.
- Connect the SX1280 busy pin to the microcontroller.
- Add test points:
  - GND
  - chip select
  - SDI
  - SDO
  - SCK
  - RESET
  - ...?
- Print version number on the PCB.
- Remove small compenents slik screen labels.
- Make it possible to manually pull up the BOOT0 pin in case the firmware disrupts SWD communication.
- The top internal layer should be a ground plane to ensure a nice return path for the RF signal. Make a 6 layer board?
  - Signal
  - GND
  - Signal
  - Power
  - Signal
  - GND
- Move motor current sensing away from PF0 and PF1 since they are not connected to any operational amplifiers.

TODO: check if MAIN_SENSE and TAIL_SENSE work!

# GPIO pins

| PA0  | NC         |
| PA1  | TAIL_CTL   |
| PA2  | LEFT_CTL   |
| PA3  | /RADIO_CS  |
| PA4  | /ACCEL_CS  |
| PA5  | SCK        |
| PA6  | SDI        |
| PA7  | SDO        |
| PA8  | NC         |
| PA9  | NC         |
| PA10 | BATT_MON   |
| PA11 | RIGHT_CTL  |
| PA12 | BACK_CTL   |
| PA13 | SWDIO      |
| PA14 | SWCLK      |
| PA15 | NC         |
| PB0  | /GNSS_CS   |
| PB3  | SWO        |
| PB4  | LED_R      |
| PB5  | LED_G      |
| PB6  | MAIN_CTL   |
| PB7  | LED_B      |
| PF0  | MAIN_SENSE |
| PF1  | TAIL_SENSE |

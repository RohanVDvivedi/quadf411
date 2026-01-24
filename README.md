# STM32F411CEU6 HAL SAMPLE
A sample project to show how to build a modular STM32 HAL-supported application for the black pill board.

## Setup instructions
**Install dependencies :**
 * `sudo apt install gcc-arm-none-eabi`
 * for uploading the binary to the board
   * `sudo apt install stm32flash` for stm32flash (over uart)
   * OR `sudo apt-get install stlink-tools` for st-flash (over stlink v2)
 * No need to worry about the rest, as they would be managed by submodules.

**Download source code :**
 * `git clone --recurse-submodules https://github.com/RohanVDvivedi/stm32f411ceu6_hal_sample.git`

**Build from source :**
 * `cd stm32f411ceu6_hal_sample`
 * `make dependencies` *only when you modify a dependency*
 * `make all` *only when you modify the project source*

**Upload the build to the board :**
 * `make upload` or `make upload-using-uart`
 * ***you may discard the build by*** `make clean`

### resources
 * [STM32F411 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
# quadf411
A quadcopter flight controller firmware project built using a stm32f411ceu6 based black pill board.
Uses
 * adxl345
 * itg3205
 * hmc5883l
 * neo6m
 * fsia6b (i-bus mode)

## Setup instructions
**Install dependencies :**
 * `sudo apt install gcc-arm-none-eabi`
 * for uploading the binary to the board
   * `sudo apt install stm32flash` for stm32flash (over uart)
   * OR `sudo apt-get install stlink-tools` for st-flash (over stlink v2)
 * No need to worry about the rest, as they would be managed by submodules.

**Download source code :**
 * `git clone --recurse-submodules https://github.com/RohanVDvivedi/quadf411.git`

**Build from source :**
 * `cd quadf411`
 * `make dependencies` *only when you modify a dependency*
 * `make all` *only when you modify the project source*

**Upload the build to the board :**
 * `make upload` or `make upload-using-uart`
 * ***you may discard the build by*** `make clean`

### resources
 * [STM32F411 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0383-stm32f411xce-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
cargo build --release
cargo objcopy --bin blink --target thumbv7m-none-eabi --release -- -O binary blink.bin
rem st-flash erase
rem st-flash write blink.bin 0x8000000
rem cargo embed --release --chip STM32F103RE
cargo embed --release --chip STM32F103C8


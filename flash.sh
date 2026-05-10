#!/bin/bash
# requires openOCD to flash
set -e  

mkdir -p build
cmake -B build -S . -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake

cmake --build build

ELF=$(find build -maxdepth 1 -name "*.elf" | head -n 1)
if [[ ! -f "$ELF" ]]; then
    echo "Error: ELF file not found in build folder"
    exit 1
fi
echo "Flashing $ELF ..."

openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "adapter speed 480" \
    -c "program $ELF verify reset exit"

echo "Flash Succesful !"
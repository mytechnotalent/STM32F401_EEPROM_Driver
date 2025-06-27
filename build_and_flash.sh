#!/bin/bash

# Build and Flash Script for STM32F401RE.
# This script builds the assembly project and flashes it to the STM32F401RE microcontroller.

# Exit on any error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Echo script information
echo -e "${YELLOW}STM32F401RE - Build and Flash Script${NC}"
echo    "=================================================="

# Change to project directory
cd project

# Clean previous build artifacts
echo -e "${YELLOW}Cleaning previous build artifacts...${NC}"
rm -f main.o main.elf main.bin

# Step 1: Assemble the main.s file
echo -e "${YELLOW}Step 1: Assembling main.s...${NC}"
arm-none-eabi-as -g src/main.s -o main.o
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Assembly completed successfully${NC}"
else
    echo -e "${RED}✗ Assembly failed${NC}"
    exit 1
fi

# Step 2: Link the object file
echo -e "${YELLOW}Step 2: Linking object file...${NC}"
arm-none-eabi-ld main.o -o main.elf -T STM32F401RETx_FLASH.ld
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Linking completed successfully${NC}"
else
    echo -e "${RED}✗ Linking failed${NC}"
    exit 1
fi

# Step 3: Create binary file
echo -e "${YELLOW}Step 3: Creating binary file...${NC}"
arm-none-eabi-objcopy -O binary --strip-all main.elf main.bin
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Binary creation completed successfully${NC}"
else
    echo -e "${RED}✗ Binary creation failed${NC}"
    exit 1
fi

# Step 4: Flash to STM32F401RE
echo -e "${YELLOW}Step 4: Flashing to STM32F401RE...${NC}"
echo "Make sure your NUCLEO-F401RE board is connected via USB!"
read -p "Press [Enter] to continue with flashing..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program main.elf verify reset exit"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Flashing completed successfully${NC}"
else
    echo -e "${RED}✗ Flashing failed${NC}"
    echo -e "${RED}Please check your board connection and try again${NC}"
    exit 1
fi
echo "================================================================"
echo -e "${GREEN}Build and flash process completed successfully!${NC}"
echo ""
echo "You can debug the application using:"
echo "  ./gdb_server.sh (in another terminal)"
echo "  ./gdb_client.sh"

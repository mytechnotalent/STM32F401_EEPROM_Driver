#!/bin/bash

#
# FILE: gdb_client.sh
#
# DESCRIPTION:
# This shell script starts the ARM GDB client for the STM32 utilizing the
# STM32x microcontroller. The GDB client connects to the OpenOCD
# server for interactive debugging sessions.
#
# AUTHOR: Kevin Thomas
# CREATION DATE: March 7, 2024
# UPDATE DATE: June 20, 2025
#

# Exit on any error
set -e 

# Echo client information
echo "STM32x GDB Client - ARM GDB Debugger"
echo "===================================="

# Change to project directory
cd project

# Check if ELF file exists
if [ ! -f "main.elf" ]; then
    echo "Error: main.elf not found. Please build the project first."
    echo "Run: ./build_and_flash.sh or build manually"
    exit 1
fi

# Echo OpenOCD server information and commands
echo "Starting ARM GDB client..."
echo "Connecting to OpenOCD server on localhost:3333"
echo ""
echo "Useful GDB commands:"
echo "  (gdb) target remote :3333    - Connect to OpenOCD server"
echo "  (gdb) monitor reset halt     - Reset and halt the MCU"
echo "  (gdb) load                   - Load the program"
echo "  (gdb) continue               - Continue execution"
echo "  (gdb) step                   - Step one instruction"
echo "  (gdb) info registers         - Show register contents"
echo "  (gdb) x/8i \$pc              - Disassemble at PC"
echo "  (gdb) break Reset_Handler    - Set breakpoint"
echo "  (gdb) quit                   - Exit GDB"
echo ""

# Start GDB with automatic connection commands
arm-none-eabi-gdb -q main.elf \
    -ex "target remote :3333" \
    -ex "monitor reset halt" \
    -ex "break main" \
    -ex "continue"

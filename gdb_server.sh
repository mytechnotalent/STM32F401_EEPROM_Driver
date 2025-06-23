#!/bin/bash

#
# FILE: gdb_server.sh
#
# DESCRIPTION:
# This shell script starts the OpenOCD GDB server for the STM32 utilizing the
# STM32x microcontroller. The GDB server provides a debug interface
# for remote debugging sessions.
#
# AUTHOR: Kevin Thomas
# CREATION DATE: March 7, 2024
# UPDATE DATE: June 20, 2025
#

# Exit on any error
set -e

# Echo server information
echo "STM32x GDB Server - OpenOCD Debug Server"
echo "========================================"

# Change to project directory
cd project

# Echo OpenOCD server information
echo "Starting OpenOCD GDB server on port 3333..."
echo "Connect with GDB client using: target remote :3333"
echo "Press Ctrl+C to stop the server"
echo ""

# Start OpenOCD GDB server with standard config
pkill -f openocd || true
pkill -f gdb || true

# Perform full MCU reset before starting server
echo "Performing full MCU reset..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init" -c "reset halt" -c "shutdown" || true
sleep 2

# Start OpenOCD GDB server
echo "Starting OpenOCD GDB server..."
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

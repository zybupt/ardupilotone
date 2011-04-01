#!/bin/bash
avarice --mkII --capture --jtag usb :4242 &
gnome-terminal -x avr-gdb /tmp/ArduPilotOne.build/ArduPilotOne.elf

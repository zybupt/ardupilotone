#!/binb/bash
port=$1
avrdude -c arduino -p m1280 -P $port -carduino -b57600 -U /tmp/ArduPilotOne.build/ArduPilotOne.hex

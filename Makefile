sketches= \
ArduPilotOne \
MavlinkTest \
SensorTest \
MouseTest \
libraries/AP_Compass/examples/AP_Compass_test

BOARD=mega
TMPDIR=${PWD}/build

all:
	for sketch in $(sketches); do echo "\nbuilding $$sketch\n"; BOARD=${BOARD} TMPDIR=${TMPDIR} make -e -C $$sketch; done
	
clean:
	for sketch in $(sketches); do echo "\ncleaning $$sketch\n"; BOARD=${BOARD} TMPDIR=${TMPDIR} make -e -C $$sketch clean; done
	
upload:
	bash ../scripts/upload
	
debug:
	bash ../scripts/debug
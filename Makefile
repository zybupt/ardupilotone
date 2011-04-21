sketches= \
ArduPilotOne \
tests/MavlinkTest \
tests/SensorTest \
tests/MouseTest \
libraries/AP_Compass/examples/AP_Compass_test \
libraries/AP_RcChannel/examples/ServoSweep \
libraries/AP_RangeFinder/examples/AP_RangeFinder_test

BOARD=mega
TMPDIR=${PWD}/build
SKETCHBOOK=${PWD}

all:
	for sketch in $(sketches); do echo "\nbuilding $$sketch\n"; BOARD=${BOARD} SKETCHBOOK=${SKETCHBOOK} TMPDIR=${TMPDIR} make -e -C $$sketch; done
	
clean:
	for sketch in $(sketches); do echo "\ncleaning $$sketch\n"; BOARD=${BOARD} SKETCHBOOK=${SKETCHBOOK} TMPDIR=${TMPDIR} make -e -C $$sketch clean; done
	
upload:
	bash ../scripts/upload
	
debug:
	bash ../scripts/debug
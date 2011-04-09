sketches= \
ArduPilotOne \
MavlinkTest \
SensorTest \
MouseTest \
libraries/AP_Compass/examples/AP_Compass_test

TMPDIR=tmp/build
BOARD=mega

all:
	for sketch in $(sketches); do echo "\nbuilding $$sketch\n"; TMPDIR=$$BOARD TMPDIR=$$PWD/build make -e -C $$sketch; done
	
clean:
	for sketch in $(sketches); do echo "\ncleaning $$sketch\n"; TMPDIR=$$BOARD TMPDIR=$$PWD/build make -e -C $$sketch clean; done
	
upload:
	bash ../scripts/upload
	
debug:
	bash ../scripts/debug
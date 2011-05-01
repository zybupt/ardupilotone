sketches= \
libraries/APO/examples/AutopilotCar \
libraries/APO/examples/AutopilotQuad \
libraries/AP_Compass/examples/AP_Compass_test \
libraries/APO/examples/ServoSweep \
libraries/APO/examples/ServoManual \
libraries/AP_RangeFinder/examples/AP_RangeFinder_test \

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
sketches= ArduPilotOne MavlinkTest

all:
	for sketch in $(sketches); do echo "\nbuilding $$sketch\n"; make -C $$sketch; done
	
clean:
	for sketch in $(sketches); do echo "\ncleaning $$sketch\n"; make -C $$sketch clean; done
	
upload:
	bash ../scripts/upload
	
debug:
	bash ../scripts/debug
SKETCHBOOK=${PWD}
TMPDIR=$(SKETCHBOOK)/build

include $(SKETCHBOOK)/config.mk

all: $(SKETCHBOOK)/config.mk
	SKETCHBOOK=$(SKETCHBOOK) TMPDIR=$(TMPDIR) make -e -C $(SKETCH_PATH)
	
clean:
	rm -rf build
	
upload:
	SKETCHBOOK=$(SKETCHBOOK) TMPDIR=$(TMPDIR) make -e -C $(SKETCH_PATH) upload
	
debug:
	SKETCHBOOK=$(SKETCHBOOK) TMPDIR=$(TMPDIR) make -e -C $(SKETCH_PATH) debug
	
configure: 
	rm $(SKETCHBOOK)/config.mk
	make $(SKETCHBOOK)/config.mk

$(SKETCHBOOK)/config.mk:
	@echo 'creating $(SKETCHBOOK)/config.mk'; \
	echo -n 'sketch [i.e. arduquad, ardurover] : '; read sketchPath; echo SKETCH_PATH=$$sketchPath > $(SKETCHBOOK)/config.mk; \
	echo -n 'port [i.e. /dev/ttyUSB0] : '; read port; echo PORT=$$port >> $(SKETCHBOOK)/config.mk; \
	echo -n 'board [i.e. mega, mega2560] : '; read board; echo BOARD=$$board >> $(SKETCHBOOK)/config.mk; \
	echo config file written to $(SKETCHBOOK)/config.mk
	cat $(SKETCHBOOK)/config.mk
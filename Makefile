SKETCHBOOK=${PWD}
TMPDIR=$(SKETCHBOOK)/build

include $(SKETCHBOOK)/config.mk

all: help

help:
	@echo please make sure you have created a config.mk file in your sketchbook similar to this:
	@echo SKETCH_PATH=arduquad
	@echo PORT=/dev/ttyUSB0
	@echo BOARD=mega2560
	@echo
	@echo then you may choose from the following make targets: 
	@echo build : compile the code
	@echo upload : upload the code to the board
	@echo debug : start a gdb JTAGICE mkII debug session
	@echo configure: prompts the user to create a config.mk file, this does not work in eclipse
	@echo
	@echo note if you are using eclipse: build, clean, debug, upload targets are listed in the
	@echo make target gui window of the project.

build: $(SKETCHBOOK)/config.mk
	SKETCHBOOK=$(SKETCHBOOK) TMPDIR=$(TMPDIR) make -e -C $(SKETCH_PATH)
	
clean:
	rm -rf build
	
upload: build $(SKETCHBOOK)/config.mk
	SKETCHBOOK=$(SKETCHBOOK) TMPDIR=$(TMPDIR) make -e -C $(SKETCH_PATH) upload
	
debug: build $(SKETCHBOOK)/config.mk
	SKETCHBOOK=$(SKETCHBOOK) TMPDIR=$(TMPDIR) make -e -C $(SKETCH_PATH) debug
	
configure:
	rm $(SKETCHBOOK)/config.mk
	@echo 'creating $(SKETCHBOOK)/config.mk'; \
	echo -n 'sketch [i.e. ardupilotone] : '; read sketchPath; echo SKETCH_PATH=$$sketchPath > $(SKETCHBOOK)/config.mk; \
	echo -n 'port [i.e. /dev/ttyUSB0] : '; read port; echo PORT=$$port >> $(SKETCHBOOK)/config.mk; \
	echo -n 'board [i.e. mega, mega2560] : '; read board; echo BOARD=$$board >> $(SKETCHBOOK)/config.mk; \
	echo config file written to $(SKETCHBOOK)/config.mk
	cat $(SKETCHBOOK)/config.mk

$(SKETCHBOOK)/config.mk:
	@echo 'creating default $(SKETCHBOOK)/config.mk, please edit'; \
	echo SKETCH_PATH=ardupilotone > $(SKETCHBOOK)/config.mk; \
	echo PORT=/dev/ttyUSB0 >> $(SKETCHBOOK)/config.mk; \
	echo BOARD=mega2560 >> $(SKETCHBOOK)/config.mk; \
	echo config file written to $(SKETCHBOOK)/config.mk
	cat $(SKETCHBOOK)/config.mk
	
.PHONY: build upload debug configure clean help all

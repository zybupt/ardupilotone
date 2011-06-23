Overview
--------------------------------------------------------------------------------------
The overall goal is a single autopilot system capable of flying a quad/ car/ heli/ boat/ plane
just by changing a compile time flag. In order to reach this goal many existing structures
have been highly abstracted and reworked.

Another goal is to abstract the sensor interfaces enough that this autopilot can
also be used for the original ArduPilot system and future DIY drones hardware.

The ArduPilot support will require further library abstraction that has not yet 
been handled.

Building with make/ eclipse.
--------------------------------------------------------------------------------------
1. on the command line type make configure. This will create a file in your sketchbook
similar to this: 
	
	example SKETCHBOOK/config.mk file:

	SKETCH_PATH=arduquad
	PORT=/dev/ttyUSB0
	BOARD=mega2560
	
Explanation of variables:

SKETCH_PATH : the relative path from the sketchbook to where the sketch you want to
	build/upload/debug is located
PORT : the device name used by avrdude for uploading the program to the board
BOARD : the board as given in the arduino boards.txt file (mega for 1280, mega2560 for the new 2560 board)
	
2. Once this file is created you can type build to compile the code, upload to upload the code to the board,
and debug to start a JTAGICE mkII debug session.

Building with arduino
--------------------------------------------------------------------------------------
1. Start the Arduino Gui.
2. Set your sketchbook to the root of the ArduPilotOne folder. Set the board to the 1280/2560
   depending on your board. Set the serial port to the device connected to the APM hardware.
3. Restart Arduino. You should now see the ardurover, arduboat, arduplane, arduquad etc projects
	listed. You should also see all of the relavent AP libraries and examples.
4. Select the sketch you would like to compile and hit the play looking button to compile it.
5. If compilation fails try to hit the compile button again, this is a bug in the arduino gui.
6. Now click the upload button.

COMPLETED: Loop Scheduling
--------------------------------------------------------------------------------------
A Loop class has been implemented to schedule callbacks at different rates. When each loop
is updated it calls all subloop updates first and then calls its own update. This allows
a hierarchical event tree to be setup. The loops currently compute dt and load so don't get
too carried away with the number of subloops. This should probably be done in a special update
function to allow these calcs to be turned off. The callback functions have a void pointer 
to allow access to the class member varibles through casting the this pointer. This loop
scheduling forms the backbone of the code and has already been completed.

COMPLETED: EEPROM Parameter Management
--------------------------------------------------------------------------------------
The AP_Var classes have been utilized.

COMPLETED: Control System
--------------------------------------------------------------------------------------
The control system is defined by a set of blocks, much like scicoslab or simulink. This block set
is then read in and loads the appropriate classes to create the control system.

COMPLETED: Guidance System
--------------------------------------------------------------------------------------
The guidance system takes the waypoints and current position as input and outputs the desired
heading, velocity, and altitude to the controller. This should be relatively easy to 
implement.

COMPLETED: Communication
--------------------------------------------------------------------------------------
The mavlink communication protocol will be used for comms at this time.

TODO: Airplane control system
--------------------------------------------------------------------------------------
The airplane control system has to be completed. This should be a minor task.

TODO: Verify quad control system
--------------------------------------------------------------------------------------
The quad control system is completed but still needs further testing.

Libraries
--------------------------------------------------------------------------------------
The existing APM libraries should provide all necessary sensor interfaces etc.

Coding Standard
--------------------------------------------------------------------------------------
The main structure of code strives to follow the C++ paradigms.
1. Resource allocation is initialization. (RAII)
2. Do not use global variables.
3. etc.. etc..

The documentation in the code is being done with doxygen javadoc style markup. This will allow
automatic generation of documentation for the code.

The coding standard is pretty well.. standard :-)
1. camelCase
2. lowercase for variables names
3. uppercase for classes
4. enums are lower case followed by _t ... example_t
5. Format the code with astyle on the K/R setting.

All are welcome to help with the code.

Cheers,

James (james.goppert@gmail.com)

// vim:ts=4:sw=4:expandtab

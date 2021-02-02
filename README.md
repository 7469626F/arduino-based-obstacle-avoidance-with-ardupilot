# arduino-based-obstacle-avoidance-with-ardupilot

This project eplains how to set up an arduino with ardupilot so that it can send ranging data to ardupilot from any sensor that the arduino can read over mavlink.

# basics

For the basic obstacle avoidance to work ardupilot requires a minimum of one distance and a maximum of ten distances. Each distance is attached to one of the ten available directions. These include eight directions in the horizontal plane. Direction 0 is straight ahead then with each increment the heading changes 45° clockwise up to number 7. The 9th and 10th  direction are upwards(designated with 24) and downwards(designated with 25) respectively.
An arduino can be used to supply these distances over mavlink. The origin of these values is not important, any sensor that can be used with an arduino or even a virtual sensor is sufficient.

# ardupilot setup

Ardupilot needs to be setup in a way that it will use mavlink as a source for the ranging data.

AVOID_ENABLE = 7     (all)

PRX_TYPE = 2     (mavlink)

I my case the arduino is hooked up to serial 2. Any free serial port will do the trick but avoid sbus specific serial ports since those may have a hardware inverter.
The baud rate needs to match the baud rate that is defined in the arduino script.

SERIAL2_PROTOCOL = 2     (mavlink2) 

SERIAL2_BAUD = 115     (115200)

# wiring

The arduino only needs two data wires in order to communicate with the flightcontroller.

Arduino TX goes to RX and the arduino RX obviously goes to the matching TX.

The used mavlink libraries don't suport softserial, this means that the flight controller has to be conected to a hardware serial port. Take this in consideration when you want to use a sensor that also requires a hardware serial port.

Al that's left now is a stable power supply for the arduino and a ground wire inbetween the arduino and the flight controller.

# arduino setup

First of all the script needs to be edited in order to match the used arduino model, sensors and configuration. 

step_1:

Confirm that the sensor works with arduino. Hook the sensor up and test it out, most sensors come with a basic example script that tests the basic functions.
This step makes integration and debugging a lot easier.

step_2:

Copy everything from the sensor example script to the relevant location in the blank script that is included with this project.

Do this for all the sensors.

step_3:

Change the mavlink message so that it matches the sensors direction and useful range.

step_4:

Instal the mavlink library. this is the library I used: https://discuss.ardupilot.org/uploads/default/original/3X/b/a/ba1fb2beffca5c496694171abab8900ae0c8e257.zip

step_5:

Wire everything up and confirm that ardupilot receives the distances.

# debug and testing



# sources
General mavlink information: https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR

Mavlink library : https://discuss.ardupilot.org/uploads/default/original/3X/b/a/ba1fb2beffca5c496694171abab8900ae0c8e257.zip

The majority of the code is based on this project: https://discuss.ardupilot.org/t/avoidance-experiments-with-the-poc-and-benewake-tfmini/25277
